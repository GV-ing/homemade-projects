# -*- coding: utf-8 -*-
import pygame
import socket
import threading
import time
import cv2
import numpy as np

# Inizializza Pygame
pygame.init()

# Configurazione finestra
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Joystick connettore ESP32 con Streaming")

# Colori
# Colori
RED = (255, 0, 0, 200)
GREEN = (0, 255, 0, 200)
BLUE = (0, 0, 255, 200)
WHITE = (255, 255, 255, 200)
BLACK = (0, 0, 0, 200)
GRAY = (169, 169, 169, 200)
YELLOW = (255, 255, 0, 200)  # Per eventuali indicatori aggiuntivi

# Variabili joystick
circle_pos = (200, 450)
circle_radius = 15
max_radius = 120
prev_x, prev_y = 0, 0
tolleranza = 0.0
smooth_factor = 0.5

# Connessione ESP32
ESP32_IP = "192.168.1.50"
ESP32_PORT = 5678
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
is_connected = False
last_button_pressed = ""
light_on = False
mod_precisione = False

# Streaming video
STREAM_URL = "http://192.168.1.212:8080/stream"
frame_surface = None

def try_connect():
    global is_connected
    try:
        client.connect((ESP32_IP, ESP32_PORT))
        is_connected = True
    except Exception as e:
        is_connected = False

def send_joystick_data(left_x=None, right_x=None, right_y=None, button=None):
    if is_connected:
        data = ""
        if button:
            data = button + "\n"
        else:
            if left_x is not None:
                data += f"LX:{left_x:.2f},"
            if right_x is not None and right_y is not None:
                data += f"RX:{right_x:.2f},RY:{right_y:.2f}\n"
        
        if data:
            try:
                client.send(data.encode('utf-8'))
            except Exception as e:
                print(f"Errore invio dati: {e}")

def control_torch():
    global light_on
    light_on = not light_on
    send_joystick_data(button="L_on" if light_on else "L_off")

def control_precision_mode():
    global mod_precisione
    mod_precisione = not mod_precisione
    send_joystick_data(button="M_on" if mod_precisione else "M_off")

def stream_thread():
    global frame_surface
    while True:
        try:
            cap = cv2.VideoCapture(STREAM_URL)
            while True:
                ret, frame = cap.read()
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = cv2.resize(frame, (SCREEN_WIDTH, SCREEN_HEIGHT))
                    frame_surface = pygame.image.frombuffer(frame.tobytes(), frame.shape[1::-1], "RGB")
                else:
                    break
                time.sleep(0.03)
        except Exception as e:
            print(f"Errore stream: {e}")
            time.sleep(1)

def game_loop():
    global circle_pos, prev_x, prev_y, last_button_pressed

    # Connessione iniziale
    try_connect()

    # Thread streaming
    streaming_thread = threading.Thread(target=stream_thread)
    streaming_thread.daemon = True
    streaming_thread.start()

    # Thread joystick
    joystick_thread = threading.Thread(target=monitor_joystick)
    joystick_thread.daemon = True
    joystick_thread.start()

    running = True
    while running:
        # Disegna sfondo
        if frame_surface:
            screen.blit(frame_surface, (0, 0))
        else:
            screen.fill(BLACK)

        # Disegna elementi UI
        draw_interface()

        # Gestione eventi
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.JOYBUTTONDOWN:
                handle_buttons(event.button)

        pygame.display.flip()
        time.sleep(0.016)

    pygame.quit()

def draw_interface():
    # Cerchio del joystick
    pygame.draw.circle(screen, GRAY, (200, 450), max_radius, 2)
    pygame.draw.circle(screen, BLUE, circle_pos, circle_radius)

    # Testo e indicatori
    font = pygame.font.Font(None, 36)
    text = font.render(f"Ultimo comando: {last_button_pressed}", True, WHITE)
    screen.blit(text, (10, 10))

    # Stato connessione
    status_color = GREEN if is_connected else RED
    pygame.draw.circle(screen, status_color, (SCREEN_WIDTH - 50, 50), 10)

def monitor_joystick():
    global circle_pos, prev_x, prev_y
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        pygame.event.pump()
        
        # Lettura assi (originale)
        right_x = joystick.get_axis(0) * 0.5  # RX
        right_y = joystick.get_axis(1)         # RY 
        left_x = joystick.get_axis(2)          # LX

        # Applica tolleranza e smoothing
        right_x = right_x if abs(right_x) > tolleranza else 0
        right_y = right_y if abs(right_y) > tolleranza else 0
        left_x = left_x if abs(left_x) > tolleranza else 0

        # Calcola posizione joystick virtuale
        new_x = 200 + (prev_x * (1 - smooth_factor) + right_x * smooth_factor) * max_radius
        new_y = 450 + (prev_y * (1 - smooth_factor) + right_y * smooth_factor) * max_radius
        
        # Limita il movimento
        distance = ((new_x - 200)**2 + (new_y - 450)**2)**0.5
        if distance > max_radius:
            ratio = max_radius / distance
            new_x = 200 + (new_x - 200) * ratio
            new_y = 450 + (new_y - 450) * ratio
        
        circle_pos = (int(new_x), int(new_y))
        prev_x, prev_y = right_x, right_y

        # Invia dati (formato originale)
        send_joystick_data(left_x=left_x, right_x=right_x, right_y=right_y)
        time.sleep(0.05)

def handle_buttons(button_id):
    global last_button_pressed
    mapping = {
        0: "A",
        1: "B",
        2: "C",
        3: "D",
        7: control_torch,
        8: control_precision_mode
    }
    
    if button_id in mapping:
        if callable(mapping[button_id]):
            mapping[button_id]()
        else:
            last_button_pressed = mapping[button_id]
            send_joystick_data(button=mapping[button_id])

if __name__ == "__main__":
    game_loop()