import serial
import matplotlib.pyplot as plt
from drawnow import drawnow
import time
from datetime import datetime

# Inicjalizacja połączenia UART w Pythonie
ser = serial.Serial('COM3', 115200, timeout=1)  # COMx - odpowiedni port COM

ser.write(b'26')
time.sleep(0.5)

# Funkcja do rysowania wykresu w czasie rzeczywistym
def draw_fig():
    plt.plot(x_data, y_data, 'ro')
    plt.title('Real-time Temperature Plot')
    plt.xlabel('Time')
    plt.ylabel('Temperature (°C)')
    plt.grid()
    with open(filename, 'a') as file:
        for i, temp in enumerate(x_data):
            file.write(f"{i+1}\t{temp}\n")



# Dane do rysowania wykresu
x_data = []
y_data = []

# Zapis danych do pliku txt
timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
filename = f"temperatura_pomiar{timestamp}.txt"

# Pętla odczytu danych z UART i rysowania wykresu
while True:
    try:
        # Odczyt danych z UART
        data = ser.readline().decode('utf-8').strip()

        # Konwersja na liczbę zmiennoprzecinkową
        temperature = float(data)

        # Dodanie danych do list
        x_data.append(len(x_data) + 1)
        y_data.append(temperature)

        # Ograniczenie ilości punktów na wykresie


        # Rysowanie wykresu w czasie rzeczywistym
        plt.figure(0, figsize=(9,6))
        drawnow(draw_fig)

    except KeyboardInterrupt:
        # Przerwanie pętli w przypadku naciśnięcia Ctrl+C
        break
    except ValueError:
        print('Jakis error')
        pass

# Zamknięcie połączenia UART
ser.close()
