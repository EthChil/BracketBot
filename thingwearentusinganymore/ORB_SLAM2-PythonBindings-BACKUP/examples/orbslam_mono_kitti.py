#!/usr/bin/env python3
import sys
import os.path

from runners import OrbslamMono


class OrbslamMonoKITTI(OrbslamMono):

    def __init__(self, vocab_path, settings_path, sequence_path):
        super().__init__(vocab_path, settings_path, sequence_path)

    def load_images(self):
        timestamps = []
        with open(os.path.join(self.sequence_path, 'times.txt')) as times_file:
            for line in times_file:
                if len(line) > 0:
                    timestamps.append(float(line))

        return [
            os.path.join(self.sequence_path, 'image_2', "{0:06}.png".format(idx))
            for idx in range(len(timestamps))
        ], timestamps


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Usage: ./orbslam_mono_kitti path_to_vocabulary path_to_settings path_to_sequence')
    OrbslamMonoKITTI(sys.argv[1], sys.argv[2], sys.argv[3]).run()
