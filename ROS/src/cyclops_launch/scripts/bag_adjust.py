#!/usr/bin/env python3
import argparse
import rosbag

class Bagger:
    FROM_TOPIC="" # must be overriden
    TO_TOPIC=None # leave as None to replace topic messages or specify new name for topic
    OUTPUT_TOPICS=None # replace with a list of strings of topics to keep
    KEEP_OLD_TOPIC=True # if TO_TOPIC is not None, this controls whether the old topic is retained
    COPY_HEADER=True # if True, extract header from the old message and apply to the new one. Set to False if no header or you want to modify it yourself
    HELP_MSG="" # Override to provide a better description for your script
        
    def __init__(self):
        if self.FROM_TOPIC=="":
            raise ValueError("FROM_TOPIC must be overriden")
        if self.HELP_MSG=="":
            help_msg = "A generic script to modify a rosbag"
        else:
            help_msf = self.HELP_MSG
        parser = argparse.ArgumentParser(description=help_msg)
        parser.add_argument('-o', '--output_file', help="File for the new bag data to be stored within", required=True)
        parser.add_argument('-b', '--bag', help="Bag file to edit", required=True)
        self.opts = parser.parse_args()

        
    def process(self, msg):
        """Override this method in descendant classes, must return a new message"""
        raise NotImplementedError()
        
    def run(self):
        with rosbag.Bag(self.opts.bag,"r") as in_bag:
            with rosbag.Bag(self.opts.output_file, "w") as out_bag:
                message_reader = in_bag.read_messages()
                for topic, msg, t in message_reader:
                    if topic == self.FROM_TOPIC:
                        new_msg = self.process(msg)
                        if self.COPY_HEADER:
                            new_msg.header = msg.header
                        if self.TO_TOPIC is None:
                            out_bag.write(self.FROM_TOPIC, new_msg, t)
                        else:
                            out_bag.write(self.TO_TOPIC, new_msg, t)
                            if self.KEEP_OLD_TOPIC:
                                out_bag.write(self.FROM_TOPIC, new_msg, t)
                    else: 
                        if self.OUTPUT_TOPICS is None or topic in self.OUTPUT_TOPICS:
                            out_bag.write(topic, msg, t)

        
