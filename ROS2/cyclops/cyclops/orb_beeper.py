import asyncio

import rclpy
from autonode import Node, subscription
from cyclops_interfaces.srv import Beep, Beep_Request, Beep_Response
from orb_slam3.msg import NumPoints

class OrbBeeper(Node):
    def __init__(self):
        super().__init__()
        self.beeper = self.create_client(Beep, "beep")

    @staticmethod
    def number_to_note(value, min_value=30, max_value=200):
        """
        Map a numeric value to a musical note between C4 and C5.
        """
        # Define notes in one octave
        notes = ["C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4", "C5"]

        # Clamp the input to stay within range
        value = max(min_value, min(max_value, value))

        # Compute position (0.0 to 1.0)
        norm = (value - min_value) / (max_value - min_value)

        # 12 semitones between C4 and C5
        total_semitones = 12
        note_index = int(norm * total_semitones)
        return notes[note_index]

    @subscription(NumPoints)
    async def num_points(self, msg: NumPoints):
        req = Beep_Request()
        if msg.count < 15:
            req.notes = ['G3']
        else:
            req.notes = [self.number_to_note(msg.count)]
        req.durations = [80]
        self.get_logger().info(f"{msg.count} -> {req.notes[0]}")
        self.beeper.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    b = OrbBeeper()
    b.run_async()

if __name__ == "__main__":
    main()