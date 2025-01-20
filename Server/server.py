# @author Ömer & Oscar & Firoz

import json 
import serial
import tkinter as tk
from serial import Serial, SerialException
import random
from collections import deque
from PIL import Image, ImageTk
import re
import threading
import queue


# === Konstanter och konfiguration ===
rows, cols = 35, 40 #antal rader och kolumner i rutnätet
cell_size = 17 #storlek på varje celler i pixlar
PORT = '/dev/cu.usbserial-0001'  #Portnamn för att koppla till serielkommunikation
BAUDRATE = 115200   #bandbredden för seriellkommunikaitonen
serial_queue = queue.Queue() # kö för att hantera inkommande data

# === Variables ===
device_positions = {}  # Mapp av enhet ID och deras positioner
device_states = {}     # Mapp av enhet ID och deras tillstånd
red_dots = [] #listan av eld
smoke_dots = [] #listan av rök
poi_dots = []  # Lista för personer som behöver hjälp
farlig_dots = []  # Lista för brandfarligt material
elapsed_time = 0 #tiden som visas på skärmen
devices_list = []  # Global lista för enheter
device_messages = {}  # för att lagra meddelanden från enheter
poi_images = {}  # för att lagra vald bild för varje POI
device_carrying_material = {}  # node_id -> bool, True om noden bär material
device_carrying_poi = {} # node_id -> POI-bild som bärs

# Regex-mönster för att matcha inkommande strängbaserade meddelanden
pos_pattern = re.compile(r'^POS:\d+,\d+,\d+$')
state_pattern = re.compile(r'^STATE:\d+,[A-Z_]+$')
devices_pattern = re.compile(r'^Devices:\s*(\d+(?:,\s*\d+)*)\s*,?$')
fire_pattern = re.compile(r'^FIRE:\d+,\d+$')
task_complete_pattern = re.compile(r'^TASK_COMPLETE:\d+,\w+,\d+,\d+$')

# === Väggar och hindren ===
walls = [
    (0, 0),(1, 0),(2, 0),(3, 0),(4, 0),(5, 0),(6, 0),(7, 0),(8, 0),(9, 0),(10, 0),(11, 0),(12, 0),(13, 0),(14, 0),(15, 0),(16, 0),(17, 0),
    (17, 1),(17, 2),(17, 3),(17, 5),(17, 4),(17, 6),(17, 7),(17, 8),(17, 9),(17, 10),(17, 11),(17, 13),(17, 14),(16, 15),(17, 15),(14, 15),
    (15, 15),(12, 15),(13, 15),(11, 15),(10, 15),(7, 15),(6, 15),(5, 15),(4, 15),(2, 15),(1, 15),(3, 15),(0, 15),(0, 14),(0, 13),(0, 12),
    (0, 11),(0, 10),(0, 9),(0, 8),(0, 7),(0, 6),(0, 5),(0, 4),(0, 3),(0, 2),(0, 1),(18, 0),(19, 0),(20, 0),(21, 0),(22, 0),(23, 0),(26, 0),
    (27, 0),(25, 0),(24, 0),(28, 0),(29, 0),(30, 0),(32, 0),(33, 0),(34, 0),(34, 1),(34, 2),(34, 3),(34, 4),(34, 5),(34, 6),(34, 7),(34, 9),
    (34, 10),(34, 11),(34, 13),(34, 14),(34, 8),(34, 15),(34, 16),(34, 17),(34, 18),(34, 19),(33, 15),(29, 15),(28, 15),(27, 15),(25, 15),(24, 15),
    (23, 15),(23, 20),(24, 20),(25, 20),(26, 20),(28, 20),(27, 20),(17, 20),(16, 20),(15, 20),(14, 20),(13, 20),(12, 20),(11, 20),(10, 20),(7, 20),
    (6, 20),(4, 20),(5, 20),(3, 20),(2, 20),(0, 20),(1, 20),(0, 19),(0, 16),(0, 21),(0, 22),(0, 24),(0, 25),(0, 26),(0, 23),(0, 27),(0, 28),(0, 29),
    (0, 30),(0, 31),(0, 33),(0, 35),(0, 34),(0, 32),(0, 36),(0, 37),(0, 38),(2, 39),(1, 39),(0, 39),(3, 39),(4, 39),(5, 39),(6, 39),(7, 39),(8, 39),
    (9, 39),(11, 39),(10, 39),(12, 39),(13, 39),(14, 39),(16, 39),(17, 39),(15, 39),(18, 39),(19, 39),(20, 39),(21, 39),(22, 39),(23, 39),(24, 39),
    (25, 39),(26, 39),(27, 39),(29, 39),(30, 39),(28, 39),(31, 39),(33, 39),(34, 39),(32, 39),(34, 38),(34, 37),(34, 36),(34, 35),(34, 34),(33, 34),
    (30, 34),(29, 34),(32, 34),(31, 34),(28, 34),(27, 34),(24, 34),(23, 34),(23, 35),(23, 36),(23, 37),(23, 38),(22, 34),(18, 34),(17, 34),(16, 34),
    (15, 34),(14, 34),(13, 34),(12, 34),(12, 35),(12, 36),(12, 37),(12, 38),(11, 34),(10, 34),(9, 34),(8, 34),(7, 34),(6, 34),(5, 34),(1, 34),(33, 29),
    (34, 29),(34, 30),(34, 31),(34, 33),(34, 32),(32, 29),(31, 29),(30, 29),(29, 29),(34, 28),(34, 25),(34, 26),(34, 24),(33, 24),(32, 24),(31, 24),
    (30, 24),(28, 23),(28, 24),(29, 24),(34, 23),(34, 22),(34, 21),(34, 20),(33, 19),(30, 19),(32, 19),(31, 19),(29, 19),(28, 19),(28, 18),(28, 17),(28, 16),
    (28, 21),(33, 5),(32, 5),(31, 5),(30, 5),(29, 5),(28, 5),(28, 6),(28, 7),(28, 8),(28, 9),(28, 10),(27, 10),(26, 10),(25, 10),(24, 10),(23, 10),(22, 10),(18, 10),
    (22, 15),(18, 15),(26, 15),(22, 20),(18, 20)
]

# === Bildhantering ===
def load_game_images():
    images = {}
    try:
        background_image = Image.open("images/flashpoint_industry.PNG")
        background_image = background_image.resize((cols * cell_size, rows * cell_size), Image.LANCZOS)
        images['background'] = ImageTk.PhotoImage(background_image)

        fire_image = Image.open("images/fire.png")
        fire_image = fire_image.resize((cell_size, cell_size), Image.LANCZOS)
        images['fire'] = ImageTk.PhotoImage(fire_image)

        fireman_image = Image.open("images/fireman.png")
        fireman_image = fireman_image.resize((int(cell_size * 1.5), int(cell_size * 1.5)), Image.LANCZOS)
        images['fireman'] = ImageTk.PhotoImage(fireman_image)

        smoke_image = Image.open("images/smoke.png")
        smoke_image = smoke_image.resize((cell_size, cell_size), Image.LANCZOS)
        images['smoke'] = ImageTk.PhotoImage(smoke_image)

        farlig_image = Image.open("images/farlig.png")
        farlig_image = farlig_image.resize((cell_size, cell_size), Image.LANCZOS)
        images['farlig'] = ImageTk.PhotoImage(farlig_image)

        poi1_image = Image.open("images/poi_1.png")
        poi1_image = poi1_image.resize((int(cell_size * 1.5), int(cell_size * 1.5)), Image.LANCZOS)
        images['poi_1'] = ImageTk.PhotoImage(poi1_image)

        poi2_image = Image.open("images/poi_2.png")
        poi2_image = poi2_image.resize((int(cell_size * 1.5), int(cell_size * 1.5)), Image.LANCZOS)
        images['poi_2'] = ImageTk.PhotoImage(poi2_image)

        poi3_image = Image.open("images/poi_3.png")
        poi3_image = poi3_image.resize((int(cell_size * 1.5), int(cell_size * 1.5)), Image.LANCZOS)
        images['poi_3'] = ImageTk.PhotoImage(poi3_image)

    except Exception as e:
        print(f"Error loading images: {e}")

    return images


# === Visualiseringshantering ===
def setup_gui():
    root = tk.Tk()
    root.title("Firefighting Simulation and Device Management")

    main_frame = tk.Frame(root)
    main_frame.pack(fill=tk.BOTH, expand=True)

    canvas = tk.Canvas(main_frame, width=cols * cell_size, height=rows * cell_size, bg="black")
    canvas.grid(row=0, column=0, rowspan=6)

    timer_label = tk.Label(main_frame, text="Timer: 00:00:00",
                           font=("Helvetica", 16), fg="white", bg="black")
    timer_label.grid(row=6, column=0, sticky="w")

    device_listbox = tk.Listbox(main_frame, height=10)
    device_listbox.grid(row=0, column=1, padx=10, pady=5, sticky="n")

    button_frame = tk.Frame(main_frame)
    button_frame.grid(row=1, column=1, padx=10, pady=5, sticky="n")

    button_refresh = tk.Button(button_frame, text="Refresh", command=lambda: updateNodes())
    button_refresh.pack(pady=5)

    message_text = tk.Text(main_frame, height=15, width=40, state=tk.DISABLED)
    message_text.grid(row=2, column=1, padx=10, pady=5, sticky="n")

    scenario_frame = tk.Frame(main_frame)
    scenario_frame.grid(row=3, column=1, padx=10, pady=5, sticky="n")

    scenario1_button = tk.Button(scenario_frame, text="Scenario 1 - Create Fire", command=create_random_fire)
    scenario1_button.pack(pady=5)

    scenario2_button = tk.Button(scenario_frame, text="Scenario 2 - Help a Person", command=scenario_hjalpa_en_person)
    scenario2_button.pack(pady=5)

    scenario3_button = tk.Button(scenario_frame, text="Scenario 3 - Add Flammable Material", command=scenario_add_flammable_material)
    scenario3_button.pack(pady=5)

    scenario4_button = tk.Button(scenario_frame, text="Scenario 4 - Random Scenario", command=scenario_random)
    scenario4_button.pack(pady=5)

    return root, canvas, timer_label, device_listbox, message_text

#försök att ansluta till serialla porten
try:
    ser = Serial(PORT, BAUDRATE, timeout=1)
except SerialException as e:
    print(f"Error connecting: {e}")
    exit(1)


# ===Bakgrundstråd för att läsa från serialla porten och placera dem i kö ===
def serial_reader_thread():
    while True:
        try:
            if ser.in_waiting > 0:
                raw_line = ser.readline()
                if raw_line:
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    if line:
                        serial_queue.put(line)
        except Exception as e:
            print("Error reading from serial:", e)
            break


# === hanterar en linje från serialla porten och tolkar json meddelande och uppdaterar enheter och tillstånds ===
def handle_serial_line(line):
    global devices_list, device_positions, device_states

    # Om strängen är tom returnera direkt
    if not line.strip():
        return

    if "From ESP32:" in line or "Message received from" in line:
        return
    if line.startswith("DEBUG:"):
        return

    # -- Försök tolka som JSON om det ser ut som JSON --
    if line.startswith("{") and line.endswith("}"):
        try:
            data = json.loads(line)
            msg_type = data.get("type", "")  # Hämta typ av meddelande

            # == position_update ==
            if msg_type == "position_update":
                # Kolla att node_id och position finns
                if "node_id" in data and "position" in data:
                    node_id = str(data["node_id"])
                    pos_obj = data["position"]
                    # Se till att x och y finns
                    if "x" in pos_obj and "y" in pos_obj:
                        x = pos_obj["x"]
                        y = pos_obj["y"]
                        device_positions[node_id] = (x, y)
                        update_firemen_positions()
                        draw_grid()
                    else:
                        print("Warning: 'position_update' saknar 'x' eller 'y':", data)
                else:
                    print("Warning: 'position_update' saknar 'node_id' eller 'position':", data)

            # == position_batch ==
            elif msg_type == "position_batch":
                # Exempel: { "type": "position_batch", "positions": [ { "node_id":123, "x": 2, "y": 5 }, ... ] }
                positions_list = data.get("positions", [])
                for position in positions_list:
                    node_id = str(position.get("node_id", "unknown"))
                    x = position.get("x")
                    y = position.get("y")
                    if x is not None and y is not None:
                        device_positions[node_id] = (x, y)
                update_firemen_positions()
                draw_grid()

            # == state_update ==
            elif msg_type == "state_update":
                if "node_id" in data and "state" in data:
                    node_id = str(data["node_id"])
                    state = data["state"]
                    device_states[node_id] = state
                    update_text_box(node_id, f"State updated to: {state}")

                    handle_state_update(node_id, state, 0, 0)
                else:
                    print("Warning: 'state_update' saknar 'node_id' eller 'state':", data)

            # == task_complete ==
            elif msg_type == "task_complete":
                # Exempel: { "type":"task_complete", "node_id":123, "result":"fire_extinguished", 
                #            "location": {"x":10,"y":4} }
                if "node_id" in data and "result" in data and "location" in data:
                    node_id = str(data["node_id"])
                    result = data["result"]
                    loc_obj = data["location"]
                    if "x" in loc_obj and "y" in loc_obj:
                        x = loc_obj["x"]
                        y = loc_obj["y"]
                        update_text_box(node_id, f"Task completed with result: {result} at ({x},{y})")
                        handle_task_completion(result, x, y, node_id)
                    else:
                        print("Warning: 'task_complete' saknar location.x eller location.y:", data)
                else:
                    print("Warning: 'task_complete' saknar 'node_id', 'result' eller 'location':", data)

            # == flammable_picked_up ==
            elif msg_type == "flammable_picked_up":
                # { "type":"flammable_picked_up","node_id":123,"location":{"x":29,"y":36} }
                if "node_id" in data and "location" in data:
                    node_id = str(data["node_id"])
                    loc_obj = data["location"]
                    if "x" in loc_obj and "y" in loc_obj:
                        x = loc_obj["x"]
                        y = loc_obj["y"]
                        if (x, y) in farlig_dots:
                            farlig_dots.remove((x, y))
                        device_carrying_material[node_id] = True
                        update_text_box(node_id, f"Picked up flammable material at ({x},{y})")
                        draw_grid()
                    else:
                        print("Warning: 'flammable_picked_up' saknar location.x eller location.y:", data)
                else:
                    print("Warning: 'flammable_picked_up' saknar 'node_id' eller 'location':", data)

            # == poi_carrying_update ==
            elif msg_type == "poi_carrying_update":
                if "node_id" in data and "position" in data and "poi_carrying" in data:
                    node_id = str(data["node_id"])
                    carrying = data["poi_carrying"]
                    loc_obj = data["position"]
                    if "x" in loc_obj and "y" in loc_obj:
                        x = loc_obj["x"]
                        y = loc_obj["y"]
                        if carrying:
                            # 1) Ta bort POI från kartan
                            if (x,y) in poi_dots:
                                poi_dots.remove((x,y))
                            # 2) Ta reda på vilken bild denna POI hade:
                            image_key = poi_images.pop((x,y), 'poi_3')
                            # 3) Koppla brandmannen till denna bild
                            device_carrying_poi[node_id] = image_key
                            update_text_box(node_id, "Now carrying POI!")
                            draw_grid()
                        else:
                            # Om brandman inte längre bär POI
                            device_carrying_poi[node_id] = False
                    else:
                        print("Warning: 'poi_carrying_update' saknar position.x eller position.y:", data)

            # Om inget av ovan stämde
            else:
                # Kan vara nåt annat JSON-meddelande vi inte hanterar
                # för att debugga -> print("Unknown JSON message type:", data)
                pass

        except json.JSONDecodeError:
            pass

    # -- Om det inte är JSON, kolla om det matchar Devices: --
    if devices_pattern.match(line):
        devices = [d.strip() for d in line[len("Devices:"):].split(',') if d.strip()]
        for device in devices:
            if device.isdigit() and device not in devices_list:
                devices_list.append(device)
                device_listbox.insert(tk.END, device)
                device_messages[device] = []
                device_positions[device] = (1, 17)  # Default position.
        # Ta bort devices som inte längre är kvar
        for device in devices_list[:]:
            if device not in devices:
                index = devices_list.index(device)
                device_listbox.delete(index)
                devices_list.remove(device)
                device_messages.pop(device, None)
                device_positions.pop(device, None)
                device_states.pop(device, None)

        update_firemen_positions()
        draw_grid()
        return

# === Hämtar alla linjer från kön och hanterar dem utan att blockera ===
def process_serial_queue():
    while not serial_queue.empty():
        line = serial_queue.get()
        handle_serial_line(line)

    # schemalägg igen
    root.after(50, process_serial_queue)

# === Hanterar slutförda uppgifter baserat på resultatet ===
def handle_task_completion(result, x, y, node_id):
    if result == "fire_extinguished":
        if (x, y) in red_dots:
            red_dots.remove((x, y))
            smoke_dots.append((x, y))
            draw_grid()
    elif result == "smoke_extinguished":
        if (x, y) in smoke_dots:
            smoke_dots.remove((x, y))
            draw_grid()
    elif result == "flammable_material_carried":
        device_carrying_material[node_id] = False
        draw_grid()
    elif result == "poi_carried":
        device_carrying_poi[node_id] = False
        draw_grid()


# === Hanterar states ändringarna för en enhet ===
def handle_state_update(node_id,state,x,y):
    if state == "CARRY_FLAMMABLE_TO_FIRETRUCK":
        if (x, y) in farlig_dots:
            farlig_dots.remove((x, y))
        device_carrying_material[node_id] = True
        draw_grid()
    elif state == "MOVE_POI_TO_FIRETRUCK":
        if (x, y) in poi_dots:
            poi_dots.remove((x, y))
        device_carrying_poi[node_id] = True
        draw_grid()
    elif state == "IDLE":
        device_carrying_material[node_id] = False
        device_carrying_poi[node_id] = False
        draw_grid()

# === Skickar en update till gateway ===
def updateNodes():
    ser.write(b'UPDATE\n')
    print("Sent UPDATE command to serial port")

# === Uppdaterar textfältet med nya meddelanden ===
def update_text_box(device, message):
    if device in device_messages:
        device_messages[device].append(message)
        if len(device_messages[device]) > 200:
            device_messages[device].pop(0)

        if device == selected_device.get():
            message_text.config(state=tk.NORMAL)
            message_text.delete(1.0, tk.END)
            for msg in device_messages[device]:
                message_text.insert("end", msg + "\n")
            message_text.see("end")
            message_text.config(state=tk.DISABLED)

# === När man väljer en enhet från listan ===
def on_device_select(event):
    selection = event.widget.curselection()
    if selection:
        index = selection[0]
        device = event.widget.get(index)
        selected_device.set(device)

        message_text.config(state=tk.NORMAL)
        message_text.delete(1.0, tk.END)
        for msg in device_messages.get(device, []):
            message_text.insert(tk.END, msg + "\n")
        message_text.see("end")
        message_text.config(state=tk.DISABLED)
    else:
        selected_device.set(None)
        message_text.config(state=tk.NORMAL)
        message_text.delete(1.0, tk.END)
        message_text.config(state=tk.DISABLED)

# === Uppdaterar brandmännens positioner ===
def update_firemen_positions():
    draw_grid()

# === Skapar en slumpmässigt brand ===
def create_random_fire():
    spawn_areas = [
        {"min_row": 1, "max_row": 34, "min_col": 1, "max_col": 39},
    ]

    for area in spawn_areas:
        fires_created = 0
        max_attempts = 20
        attempts = 0

        while fires_created < 1 and attempts < max_attempts:
            random_pos = (
                random.randint(area["min_row"], area["max_row"]),
                random.randint(area["min_col"], area["max_col"])
            )

            if (random_pos not in walls and
                random_pos not in red_dots and
                random_pos not in smoke_dots and
                random_pos not in poi_dots and
                random_pos not in farlig_dots):

                red_dots.append(random_pos)
                print(f"New fire created at {random_pos}")
                # Skicka info till master
                msg = f"FIRE:{random_pos[0]},{random_pos[1]}\n"
                ser.write(msg.encode('utf-8'))
                print(f"Sent fire position to master: {msg.strip()}")
                fires_created += 1

            attempts += 1

    draw_grid() #rita elden


# === Skapar en person i kartan och skickar infon till gateway ===

def scenario_hjalpa_en_person():
    spawn_areas = [
        {"min_row": 1, "max_row": 34, "min_col": 1, "max_col": 39},
    ]

    for area in spawn_areas:
        person_added = 0
        max_attempts = 20
        attempts = 0

        while person_added < 1 and attempts < max_attempts:
            random_pos = (
                random.randint(area["min_row"], area["max_row"]),
                random.randint(area["min_col"], area["max_col"])
            )

            if (random_pos not in walls and
                random_pos not in red_dots and
                random_pos not in smoke_dots and
                random_pos not in poi_dots and
                random_pos not in farlig_dots):

                poi_dots.append(random_pos)
                
                # Slumpa en bild för personen
                image_key = 'poi_3'
                poi_images[random_pos] = image_key

                print(f"New person added at {random_pos} with image {image_key}")
                # Skicka info till master
                msg = f"POI:{random_pos[0]},{random_pos[1]},{image_key}\n"
                ser.write(msg.encode('utf-8'))
                print(f"Sent POI position to master: {msg.strip()}")
                person_added += 1

            attempts += 1

    draw_grid()


# === Lägger till ett slumpmässigt brandfarligt material ===
def scenario_add_flammable_material():
    spawn_areas = [
        {"min_row": 1, "max_row": 34, "min_col": 1, "max_col": 39},
    ]

    for area in spawn_areas:
        material_created = 0
        max_attempts = 20
        attempts = 0

        while material_created < 1 and attempts < max_attempts:
            random_pos = (
                random.randint(area["min_row"], area["max_row"]),
                random.randint(area["min_col"], area["max_col"])
            )

            if (random_pos not in walls and
                random_pos not in red_dots and
                random_pos not in smoke_dots and
                random_pos not in poi_dots and
                random_pos not in farlig_dots):

                farlig_dots.append(random_pos)
                print(f"New flammable material created at {random_pos}")
                # Skicka info till master
                msg = f"FLAMMABLE:{random_pos[0]},{random_pos[1]}\n"
                ser.write(msg.encode('utf-8'))
                print(f"Sent flammable material position to master: {msg.strip()}")
                material_created += 1

            attempts += 1

    draw_grid()

# === Ömer: Random scenario men används inte än, ska börja med den snart! ===
def scenario_random():
    pass

# === Ritar hela spelet med alla objekt: väggar, eldar, rök osv ===
def draw_grid():
    canvas.delete("all")
    if background_photo:
        canvas.create_image(0, 0, anchor="nw", image=background_photo)

    # Väggar
    for (row, col) in walls:
        x1, y1 = col * cell_size, row * cell_size
        x2, y2 = x1 + cell_size, y1 + cell_size
        canvas.create_rectangle(x1, y1, x2, y2, fill="white", outline="")

    # Eldar
    if images.get('fire'):
        for (row, col) in red_dots:
            offset_x = (cell_size - images['fire'].width()) // 2
            offset_y = (cell_size - images['fire'].height()) // 2
            fire_x = col * cell_size + offset_x
            fire_y = row * cell_size + offset_y
            canvas.create_image(fire_x, fire_y, anchor="nw", image=images['fire'])

    # Rök
    if images.get('smoke'):
        for (row, col) in smoke_dots:
            offset_x = (cell_size - images['smoke'].width()) // 2
            offset_y = (cell_size - images['smoke'].height()) // 2
            smoke_x = col * cell_size + offset_x
            smoke_y = row * cell_size + offset_y
            canvas.create_image(smoke_x, smoke_y, anchor="nw", image=images['smoke'])

    # Brandfarligt material
    if images.get('farlig'):
        for (row, col) in farlig_dots:
            offset_x = (cell_size - images['farlig'].width()) // 2
            offset_y = (cell_size - images['farlig'].height()) // 2
            farlig_x = col * cell_size + offset_x
            farlig_y = row * cell_size + offset_y
            canvas.create_image(farlig_x, farlig_y, anchor="nw", image=images['farlig'])

    # POI
    if images.get('poi_1'):
        for (row, col) in poi_dots:
            image_key = poi_images.get((row, col), 'poi_3')
            if images.get(image_key):
                poi_image = images[image_key]
            else:
                poi_image = images['poi_3']
            offset_x = (cell_size - poi_image.width()) // 2
            offset_y = (cell_size - poi_image.height()) // 2
            poi_x = col * cell_size + offset_x
            poi_y = row * cell_size + offset_y
            canvas.create_image(poi_x, poi_y, anchor="nw", image=poi_image)

    # Brandman
    if images.get('fireman'):
        for device_id, position in device_positions.items():
            offset_x = (cell_size - images['fireman'].width()) // 2
            offset_y = (cell_size - images['fireman'].height()) // 2
            player_x = position[1] * cell_size + offset_x
            player_y = position[0] * cell_size + offset_y
            canvas.create_image(player_x, player_y, anchor="nw", image=images['fireman'])

            # Om denna device bär material, rita materialet ovanpå
            if device_carrying_material.get(device_id, False):
                mat_offset_x = (cell_size - images['farlig'].width()) // 2
                mat_offset_y = (cell_size - images['farlig'].height()) // 2
                mat_x = position[1] * cell_size + mat_offset_x
                mat_y = position[0] * cell_size + mat_offset_y
                canvas.create_image(mat_x, mat_y, anchor="nw", image=images['farlig'])

            # Om denna device bär en POI, rita personen
            if device_carrying_poi.get(device_id, False):
                carried_image_key = device_carrying_poi[device_id]  
                carried_image = images.get(carried_image_key, images['poi_3'])
                canvas.create_image(player_x, player_y, anchor="nw", image=carried_image)

# === Uppdaterar timern ===
def update_timer():
    global elapsed_time
    elapsed_time += 1
    hours = elapsed_time // 3600
    minutes = (elapsed_time % 3600) // 60
    seconds = elapsed_time % 60
    timer_label.config(text=f"Timer: {hours:02}:{minutes:02}:{seconds:02}")
    root.after(1000, update_timer)

# === Huvudfunktionen som sätter upp GUI, bilder osv ===
def main():
    global root, canvas, timer_label, images, device_listbox, selected_device, message_text
    global background_photo, fire_photo, fireman_photo, smoke_photo, farlig_photo, poi1_photo, poi2_photo, poi3_photo

    root, canvas, timer_label, device_listbox, message_text = setup_gui()
    selected_device = tk.StringVar()
    selected_device.set(None)

    # Starta en bakgrundstråd för seriell läsning
    threading.Thread(target=serial_reader_thread, daemon=True).start()
    # Starta den periodiska könsprocesseringen i huvudtråden
    root.after(50, process_serial_queue)

    images = load_game_images()
    background_photo = images.get('background')
    fire_photo = images.get('fire')
    fireman_photo = images.get('fireman')
    smoke_photo = images.get('smoke')
    farlig_photo = images.get('farlig')
    poi1_photo = images.get('poi_1')
    poi2_photo = images.get('poi_2')
    poi3_photo = images.get('poi_3')

    device_listbox.bind('<<ListboxSelect>>', on_device_select)

    update_firemen_positions()
    draw_grid()
    update_timer()
    root.mainloop()

if __name__ == "__main__":
    main()
