import csv

class Logger:
    def __init__(self):
        self.topics = {}
        self.file = open(filename, 'w')
        self.csv_writer = csv.writer(self.file)
        
    def log(self, topic, *data):
        self.get_writer().writerow(data)

    def get_writer(self, topic):
        _, writer = self.topics.get(topic, [None, None])

        if writer is None:
            f = open(filename, 'w')
            writer = csv.writer(f)
            self.topics[topic] = (f, writer)

        return writer

    def exit(self):
        for f, _ in self.topics.values():
            f.close()