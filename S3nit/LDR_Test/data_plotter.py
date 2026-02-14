import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def main():
    port = input("Enter the port (Default COM14): ")
    if port.strip() == "":
        port = "COM14"
    
    try:
        ser = serial.Serial(port, 9600, timeout=1)
    except Exception as e:
        print(f"Error: {e}")
        return

    sample_count = 40 
    xData, yData = [], []

    print("Reading data...")
    while len(yData) < sample_count:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line: continue
        try:
            y = float(line)
            yData.append(y)
            xData.append(len(yData) - 1)
            print(f"[{len(yData)}/{sample_count}] Value: {y}")
        except ValueError:
            continue
    ser.close()

    fig, ax = plt.subplots(figsize=(10, 5))
    plt.subplots_adjust(bottom=0.25) 

    line_plot, = ax.plot(xData, yData, color='blue', marker='.', markersize=2, linewidth=0.8, label='Sensor Value')
    threshold_line = ax.axhline(500, color='red', linestyle='--', linewidth=1, label='Threshold')

    spans = []

    def update_background(threshold):
        for s in spans:
            s.remove()
        spans.clear()
        
        for i in range(1, len(yData)):
            is_white = yData[i] > threshold
            color = 'lightgray' if is_white else 'black'
            alpha = 0.15 if is_white else 0.4
            
            rect = ax.axvspan(xData[i-1], xData[i], color=color, alpha=alpha)
            spans.append(rect)
        fig.canvas.draw_idle()

    ax_slider = plt.axes([0.2, 0.1, 0.6, 0.03])
    slider = Slider(ax_slider, 'Threshold', 0, 1023, valinit=500, valfmt='%0.0f')

    def on_slider_move(val):
        threshold_line.set_ydata([val, val])
        update_background(val)

    slider.on_changed(on_slider_move)
    
    update_background(500)

    ax.set_title("Analysis")
    ax.set_ylim(0, 1050) 
    ax.legend(loc='upper right')
    ax.set_xlabel("Sample")
    ax.set_ylabel("Value")
    plt.show()

if __name__ == "__main__":
    main()
