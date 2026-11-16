from userDirection import HeadingReader
import csv
from datetime import datetime

reader = HeadingReader()

d = reader.get_detail()

with open("heading_log.csv","a",newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        datetime.now(),
        d["heading"],
        d["std_deg"],
        d["cal"],
        reader.sensor.magnetic,
        reader.sensor.quaternion
    ])

reader.close()