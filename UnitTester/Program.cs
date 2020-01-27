using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NMEA_Parser;
using System.IO;
using System.IO.Ports;

namespace UnitTester
{
    class Program
    {
        static void Main(string[] args)
        {
            NmeaDevice uut = new NmeaDevice(new SerialPort("COM7", 115200), @"C:\Users\Adam\Downloads\trash\uut_DIRECT.raw", @"C:\Users\Adam\Downloads\trash\uut.parerr", "uut");
            uut.MessageReceived += Uut_MessageReceived;
            uut.OpenPort();
            Console.ReadLine();
            uut.Close();
        }

        private static List<NmeaMessage> list = new List<NmeaMessage>();
        private static void Uut_MessageReceived(object sender, EventArgs e)
        {

            NmeaDevice device = (NmeaDevice)sender;
            NmeaMessage message = ((NmeaMessageReceivedEventArgs)e).Message;

            Console.WriteLine(message.ToString());
            list.Add(message);
        }

    }
}
