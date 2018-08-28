#!/usr/bin/python
# uses pexpect to open a REPL, and then interact with it
# adapted from https://arduino.stackexchange.com/a/17529


from __future__ import print_function, division, absolute_import
import random
#import pexpect.fdpexpect as pexpect
import serial
import time
import sys
import re
if sys.hexversion > 0x02ffffff:
    import tkinter as tk
else:
    import Tkinter as tk


class App(tk.Frame):
    np_default = 'Num Pulses: {}'
    integral_default = 'Integral: {}'
    def __init__(self, parent, title, serial_port):
        tk.Frame.__init__(self, parent)
        self.serial_port = serial_port
        self.npoints = 2048
        self.Line1 = [0 for x in range(self.npoints)]
        self.Line2 = [0 for x in range(self.npoints)]
        parent.wm_title(title)
        parent.wm_geometry("800x400")
        self.canvas = tk.Canvas(self, background="white")
        self.canvas.bind("<Configure>", self.on_resize)
        self.canvas.create_line((0, 0, 0, 0), tag='X', fill='darkblue', width=1)
        self.canvas.create_line((0, 0, 0, 0), tag='Y', fill='darkred', width=1)
        self.canvas.create_line((0, 0, 0, 0), tag='Z', fill='darkgreen', width=1)
        self.canvas.grid(sticky="news", columnspan=7, row=0)

        self.canvas2 = tk.Canvas(self, background="white")
        self.canvas2.bind("<Configure>", self.on_resize)
        self.canvas2.create_line((0, 0, 0, 0), tag='X', fill='darkblue', width=1)
        self.canvas2.create_line((0, 0, 0, 0), tag='Y', fill='darkred', width=1)
        self.canvas2.create_line((0, 0, 0, 0), tag='Z', fill='darkgreen', width=1)
        self.canvas2.grid(sticky="news", columnspan=7, row=1)

        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid(sticky="news")
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(0, weight=1)
        
        # change the command from self.serial_read to self.fake to debug/test the GUI/plotting
        #b = tk.Button(self, text="pulse", width=10, command=self.fake).grid(row=2,column=0)
        b = tk.Button(self, text="pulse", width=10, command=self.read_serial).grid(row=2,column=0)
        self.np = tk.StringVar(); self.np.set(self.np_default.format("None Yet"))
        self.integral = tk.StringVar(); self.integral.set(self.integral_default.format("None Yet"))
        tk.Label(self, textvariable=self.np).grid(row=2, column=1, sticky="w")
        tk.Label(self, textvariable=self.integral).grid(row=2, column=3, sticky="w")

        tk.Label(self, text="Period").grid(row=3, column=0,sticky="e")
        self.p = tk.Entry(self)
        self.p.grid(row=3, column=1,sticky="w")
        self.p.insert(0, '150')

        tk.Label(self, text="Width").grid(row=3, column=2,sticky="w")
        self.w = tk.Entry(self)
        self.w.grid(row=3, column=3,sticky="w")
        self.w.insert(0,'1')

        tk.Label(self, text="Num Pulse Pairs").grid(row=3, column=4)
        self.n_pulses = tk.Entry(self)
        self.n_pulses.grid(row=3, column=5)
        self.n_pulses.insert(0,'2')
        
        
    def fake(self):
        #d = [0, 0, 0, 0, 0, 0, 0, 54.0, 77.0, 135.0, 167.0, 184.0, 235.0, 248.0, 361.0, 246.0, 312.0, 288.0, 292.0, 269.0, 265.0, 295.0, 304.0, 284.0, 277.0, 269.0, 307.0, 306.0, 236.0, 277.0, 269.0, 307.0, 292.0, 280.0, 270.0, 310.0, 307.0, 285.0, 276.0, 270.0, 309.0, 252.0, 280.0, 272.0, 314.0, 306.0, 310.0, 254.0, 176.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
        d = [int(random.random()*1024) for i in range(self.npoints)]
        for s in d:
            self.append_value(s)
            self.append_value2(s)
        self.after_idle(self.replot)

    def on_resize(self, event):
        self.replot()

    serial_buffer = ''
    def _expect(self, serial_obj, expect):
        while not self.serial_buffer.endswith(expect):
            c = serial_obj.read(1).decode()
            self.serial_buffer += c
            #print(self.serial_buffer)
        temp = self.serial_buffer
        self.serial_buffer  = ''
        return temp

    def read_serial(self):
        """
        Check for input from the serial port. On fetching a line, parse
        the sensor values and append to the stored data and post a replot
        request.
        """
        self.Line1 = [0 for x in range(self.npoints)]
        self.Line2 = [0 for x in range(self.npoints)]

        # open the command passed in from the command-line
        sp=serial.Serial(self.serial_port)
        #pexpect.fdspawn(sp.fileno)
        
        # log output to the command-line
        #a.logfile=sys.stdout
        if self.serial_port.startswith('picocom'):
            a.expect('Terminal ready')
        time.sleep(1)
        sp.write(b'\r')
        self._expect(sp, '>>>')
        print('got REPL')
        # send CTRL-D to refresh the filesystem
        print('sending CTRL-D')
        sp.write(b'^d\r')
        #a.sendline('\r\n')
        self._expect(sp, '>>>')

        # call a (adjust) function
        sp.write(bytes('a({},{},{})\r'.format(self.p.get(), self.w.get(), self.n_pulses.get()).encode()))
        self._expect(sp, '>>>')

        # reset ADC array
        ##a.sendline('reset_vals()\r')
        ##a.expect('>>>')

        # call pulse function
        sp.write(b'pulse()\r')
        self._expect(sp, '>>>')

        # print the adc_vals
        # this seems to take a try or two for some reason... 
        # TODO add timeout for this while-loop... so it doesn't lock-up the GUI in case things break for some reason
        matches = None
        while not matches:
            sp.write(b'adc_vals\r')
            vals = self._expect(sp, '>>>')
            #print('a.before adc_vals: {}'.format(a.before))
            #print('a.after adc_vals: {}'.format(a.after))
            # parse the adc_vals list from the terminal
            matches = re.match(r'[^\[]*\[([^\]]+)\]', vals)
            if matches:
                csv = matches.groups()[0]
                new_list = [int(s.strip()) for s in csv.split(',')]
                #print('before ({})'.format(new_list))
                flip=1
                for s in new_list:
                    if flip:
                        self.append_value(s)
                        flip=0
                    else:
                        self.append_value2(s)
                        flip=1
                self.after_idle(self.replot)
                non_zeroes = [i for i in new_list if i>0]
                self.integral.set(self.integral_default.format(sum(non_zeroes)))
        sp.close()

    def append_value(self, x):
        """
        Update the cached data lists with new sensor values.
        """
        self.Line1.append(float(x))
        # remove the first item,
        #self.Line1 = self.Line1[-1 * self.npoints:]
        self.Line1.pop(0)
        return

    def append_value2(self, x):
        """
        Update the cached data lists with new sensor values.
        """
        self.Line2.append(float(x))
        # remove the first item,
        #self.Line1 = self.Line1[-1 * self.npoints:]
        self.Line2.pop(0)
        return

    def replot(self):
        """
        Update the canvas graph lines from the cached data lists.
        The lines are scaled to match the canvas size as the window may
        be resized by the user.
        """
        w = self.winfo_width()
        h = self.winfo_height()/2
        max_all = 1024 # 1e-5 # max(self.Line1) + 1e-5
        
        coordsX = []
        for n in range(0, self.npoints):
            x = (w * n) / self.npoints
            coordsX.append(x)
            coordsX.append(h - ((h * (self.Line1[n]+100)) / max_all))
        self.canvas.coords('X', *coordsX)

        coordsXX = []
        for n in range(0, self.npoints):
            x = (w * n) / self.npoints
            coordsXX.append(x)
            coordsXX.append(h - ((h * (self.Line2[n]+100)) / max_all))
        self.canvas2.coords('X', *coordsXX)


def main(args = None):
    if args is None:
        args = sys.argv

    if len(args) == 2:
        serial_port = '{}'.format(args[1])
        root = tk.Tk()
        print(serial_port)
        app = App(root, "Culture Shock!", serial_port)
        ##app.fake()  #used for debugging the GUI and plot function, without running the pulser
        app.mainloop()
        return 0
    else:
        print('usage: tk_adc_gui.py <your serial terminal program> /dev/path_to_you_serial_port\nexample: tk_adc_gui.py picocom /dev/ttyACM0')
        return -1

if __name__ == '__main__':
    sys.exit(main())