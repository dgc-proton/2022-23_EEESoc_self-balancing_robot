import serial
from sys import exit
from time import sleep
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    ser = serial.Serial("/dev/ttyACM0", 9600)
    print(f"Details of connection: \n{ser}")
    print(f"Connection open: {ser.is_open} \n\n")
    ser.flushInput
    data = bytes()

    print("**Capturing data**")
    data = capture_data()
    
    print("\nClosing connection")
    ser.close()
    print(f"Connection open: {ser.is_open}")
    print("Converting and saving data")

    # Add headings to the data
    headings = "Time_(ms), Angle_(rad), Prev_Angle, Error, Error_Sum, Kp, Ki, Kd, Power_Value\n"
    for i, byte in enumerate(bytearray(headings, "ascii")):
        data.insert(i, byte)

    with open("serial_output.csv", "wb") as file:
        file.write(bytearray(data))

    # plot graph
    data_to_graph("serial_output.csv")

    

def capture_data():
    data = bytearray()
    with serial.Serial("/dev/ttyACM0", 9600) as ser:
        while True:
            try:
                data += ser.read()
            except KeyboardInterrupt:
                break
    # process the data    
    counter = 0
    last_nl_position = 0 
    for i, byte in enumerate(data):
        # remove any bytes that are not deemed valid output from the microcontroller
        if not (0 <= int(byte) <= 127):
            del data[i]
        # remove the initial messy data
        if (int(byte) == 10):     # and data[i+1] == "r" and data[i+2] == "\\" and data[i+3] == "n"
            counter += 1
            last_nl_position = i
            if counter == 50:
                del data[:i+5]
    # delete after the last newline to eliminate incomplete data
    del data[last_nl_position:]   
    return data


def data_to_graph(filename):
    #import excel data into its own data frame
    df_xls = pd.read_csv (filename, sep=",", header=0, index_col=False, engine='python', skipinitialspace=True) 
    print(df_xls.columns)
    # plot graph
    plt.rcParams.update({'font.size': 22})
    fig, axs = plt.subplots(1, layout="constrained")
    axs.plot(df_xls["Time_(ms)"], df_xls["Error"], linewidth=1, color="b", label="Error")
    axs.plot(df_xls["Time_(ms)"], (df_xls["Angle_(rad)"] ), linewidth=1, color="g", label="Angle")
    axs.plot(df_xls["Time_(ms)"], df_xls["Error_Sum"], linewidth=1, color="r", label="Error Sum")
    axs.plot(df_xls["Time_(ms)"], df_xls["Kp"], linewidth=1, color="c", label="Kp")
    axs.plot(df_xls["Time_(ms)"], df_xls["Ki"], linewidth=1, color="m", label="Ki")
    axs.plot(df_xls["Time_(ms)"], df_xls["Kd"], linewidth=1, color="k", label="Kd")
    axs.plot(df_xls["Time_(ms)"], (df_xls["Power_Value"]), linewidth=1, color="y", label="Power Value")
    axs.set_xlabel("Time", fontweight="bold")
    axs.set_ylabel("", fontweight="bold")
    #axs.set_ylim(0) 
    #axs.set_xlim(0)
    axs.legend()
    plt.show()

if __name__ == "__main__":
    main()


"""
    data = ser.read(size=1000)

    ser.close()
    print(f"Connection open: {ser.is_open}")

    if data:
        print("Data is now being written to file")
        try:
            decoded = str(data)
        except:
            exit("Some sort of exception occurred while converting the data")
        try:
            with open("serial_output.csv", "w") as file:
                file.write(decoded)
        except:
            exit("Some sort of exception occurred while writing file")
        print("Data written to file successfully")
    else:
        print("No data was read")


    """


"""
try:
        while True:
            data += ser.readline()
    except KeyboardInterrupt:
        ser.close()
        print(f"\nConnection open: {ser.is_open}")
        if (len(data) > 100):
            print("Data is now being written to file")
            try:
                str_data = str(data.decode("utf-8", "ignore"))
            except:
                exit("Some sort of exception occurred while converting the data to string")
            
            # Decode the data
            decoded = str()
            counter = 0
            # remove the leading 2 characters
            for i, letter in enumerate(str_data):
                if (letter == "\\" and str_data[i+1] == "r" and str_data[i+2] == "\\" and str_data[i+3] == "n"):
                    counter += 1
                    if counter == 10:
                        clipped_data = str_data[i+5:]
                        break
            # insert the newline characters
            if clipped_data:
                clipped_data.replace(r"\\r\\n", "\n")
            else:
                exit("Error: data was not decoded successfully")

            # save the data
            try:
                with open("serial_output.csv", "w") as file:
                    file.write(clipped_data)
            except:
                exit("Some sort of exception occurred while writing file")
            print("Data written to file successfully")
        else:
            print("No, or insufficient, data was read! Try re-plugging in the usb cable to Arduino.")
            exit()
    """
