import csv
import os

import rospy

class Logger:
    def __init__(self):
        # self.folder = os.environ.get("LOG_FOLDER", "csv_logs")
        self.folder = rospy.get_param("log_folder", "csv_logs")
        self.topics = {}
        
    def log(self, topic, *data):
        self.get_writer(topic).writerow(
            (rospy.get_rostime(),) + data
        )

    def get_writer(self, topic):
        _, writer = self.topics.get(topic, [None, None])

        if writer is None:
            f = open(os.path.join(self.folder, topic+".csv"), 'w')
            writer = csv.writer(f)
            self.topics[topic] = (f, writer)

        return writer

    def init_topic(self, topic, subfolder="", header=None):
        f = open(os.path.join(self.folder, subfolder, topic+".csv"), 'w')
        writer = csv.writer(f)
        self.topics[topic] = (f, writer)

        if header is not None:
            writer.writerow(header)

    def exit(self):
        for f, _ in self.topics.values():
            f.close()