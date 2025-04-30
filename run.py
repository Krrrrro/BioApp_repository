import gui
import mqtt
from tkinter import messagebox
import xml.etree.ElementTree as ET
DATA_DIR = "data"
CONFIG_FILE = "config.xml"


def read_config():
    try:
        tree = ET.parse(CONFIG_FILE)
        root = tree.getroot()
        broker = root.find("broker").text
        port = int(root.find("port").text)
        topics = [topic.text for topic in root.find("topics").findall("topic")]
        if not topics:
            raise ValueError("No topics found in config")
        return broker, port, topics
    except FileNotFoundError:
        messagebox.showerror("Error", f"Config file {CONFIG_FILE} not found")
        return None, None, None
    except Exception as e:
        messagebox.showerror("Error", f"Failed to parse config: {e}")
        return None, None, None
    
def main():
    broker, port, topics = read_config()
    print(broker)
    mqtt.__Init__(broker, port, topics)
    gui.__init__(mqtt.connect_mqtt, mqtt.disconnect_mqtt)
    
if __name__ == "__main__":
    main()

    