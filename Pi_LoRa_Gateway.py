from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD
import binascii
import os
import time
import sys

BOARD.setup()


class Node_Data():
    def __init__(self, id: bytes, crc: bytes, messager: bytes):
        self.node_id = id
        self.crc_code = crc
        self.data = messager
        self.out_data = ""

    def TX_string(self, data: str):
        self.out_data = data


class LoRaRcvCont(LoRa):
    gateway_id = 0xe1

    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.set_freq(433.0)

    def start(self):
        print("Pi LoRaWan Gateway Start!")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            sleep(.5)
            rssi_value = self.get_rssi_value()
            status = self.get_modem_status()
            sys.stdout.flush()

    def on_rx_done(self):
        print("\nReceived: ")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        print(payload)
        print(bytes(payload).decode("utf-8", 'ignore'))
        if payload[0] == self.gateway_id:
            get_data = Node_Data(payload[1], bytes(payload[2:6]), bytes(payload[6:]))
            print("checking CRC")
            data_crc = bytearray(hex(binascii.crc32(get_data.data))[2:].encode())
            if len(data_crc) < 8:
                for i in range(8 - len(data_crc)):
                    data_crc = bytearray("0".encode()) + data_crc

            get_data.crc_code = binascii.b2a_hex(get_data.crc_code)
            get_data.data = get_data.data[:-2].decode("utf-8", 'ignore')
            if data_crc == get_data.crc_code:
                print("CRC check success")
                if get_data.data == "KA" or get_data.data == "READY":
                    get_data.TX_string("CMD_AC")
                    self.lora_send(get_data)
                    print("Sand CMD_AC")
                elif get_data.data == "GW":
                    print("Waiting for GPS")
                elif get_data.data == "GK":
                    print("GPS Ready")

                else:
                    get_data.TX_string("RECEIVE")
                    self.save_data(get_data)
                    self.lora_send(get_data)
                    print("Sand RECEIVE")
            else:
                print("CRC check fail")
                print("CRC: ", get_data.crc_code)
                print("Data_CRC: ", data_crc)

        else:
            print("Message not for me")
            print("Local_ID: ", self.gateway_id)
            print("Message_ID: ", payload[0])

        sleep(0.5)
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        print("\nSandout: ")
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def lora_send(self, output_data: Node_Data):
        TX_data = bytes([output_data.node_id]) + bytes([self.gateway_id]) + bytes([1]) + bytes(
            [len(output_data.out_data)]) + output_data.out_data.encode()
        self.write_payload(list(TX_data))
        self.set_mode(MODE.TX)

    def save_data(self, data: Node_Data):
        n_id = str(data.node_id)
        n_data = data.data

        # open file to save sensor data
        print("Creat sensor data dir")
        today = str(time.localtime()[0])
        for date in time.localtime()[1:3]:
            today += "-"
            today += str(date)

        print("open sensor dir")
        # save file
        os.system('[ ! -d "/home/pi/sensor_data/" ] && mkdir "/home/pi/sensor_data/"')
        os.system('[ ! -d "/home/pi/sensor_data/' + n_id + '" ] && mkdir "/home/pi/sensor_data/' + n_id + '"')
        dir_path = "/home/pi/sensor_data/" + n_id + "/"
        save_path = dir_path + today
        print(save_path)
        data_file = open(save_path, 'a')
        print(data_file)

        # check data is different then last data
        temp_path = dir_path + "tmp"
        open(temp_path, "a").close()
        temp_file = open(temp_path, "r")
        old_data = temp_file.read()
        old_data = old_data[:-1]
        temp_file.close()
        if old_data == n_data:
            return 0

        # get date string for saving the save time
        print("Getting date & time")
        time_str = str(time.localtime()[3])
        for now_time in time.localtime()[4:6]:
            time_str += ":"
            time_str += str(now_time)

        # murge sensor data and saving time
        write_date = today + " " + time_str + "  "
        print("Saveing data")
        data_file.write(write_date)
        data_file.write(n_data + "\r\n")
        data_file.close()

        # save last data to temp file
        temp_file = open(temp_path, 'w')
        temp_file.write(n_data + "\r\n")
        temp_file.close()


lora = LoRaRcvCont(verbose=False)
lora.set_mode(MODE.STDBY)

lora.set_pa_config(pa_select=1)

try:
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("")
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
