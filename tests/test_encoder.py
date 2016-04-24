#! /usr/bin/env python
from __future__ import division
PKG='test_encoder'

import unittest
from diff_drive.encoder import Encoder


class TestEncoder(unittest.TestCase):

    def setUp(self):
        self.encoder = Encoder()
        
    def testInitialization(self):
        self.assertEquals(self.encoder.getDelta(), 0)

    def testClearedDelta(self):
        self.encoder.update(100)
        self.assertEquals(self.encoder.getDelta(), 100)
        self.assertEquals(self.encoder.getDelta(), 0)

    def testIncrement(self):
        self.encoder.update(100)
        self.assertEquals(self.encoder.getDelta(), 100)

        self.encoder.update(50)
        self.assertEquals(self.encoder.getDelta(), -50)

    def testWraparound(self):
        defaultRange = 32767 - (-32768) + 1
        self.encoder.update(20000)
        self.assertEquals(self.encoder.getDelta(), 20000)

        # Wrap around the high end.
        self.encoder.update(-20000)
        self.assertEquals(self.encoder.getDelta(),
                          (-20000) + defaultRange - 20000)

        # Wrap around the low end.
        self.encoder.update(20000)
        self.assertEquals(self.encoder.getDelta(),
                          20000 - defaultRange - (-20000))

    def testCustomRange(self):
        self.encoder.setRange(0, 999)
        self.encoder.update(500)
        self.assertEquals(self.encoder.getDelta(), 500)

        self.encoder.update(900)
        self.assertEquals(self.encoder.getDelta(), 400)

        # Wrap around the high end.
        self.encoder.update(100)
        self.assertEquals(self.encoder.getDelta(), 200)

        # Wrap around the low end.
        self.encoder.update(900)
        self.assertEquals(self.encoder.getDelta(), -200)


if __name__ == '__main__':
    unittest.main()
