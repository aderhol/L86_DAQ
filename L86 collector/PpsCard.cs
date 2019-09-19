﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using CustumLoggers;
using NMEA_Parser;

namespace PpsCardDelivery
{
    class PpsCard
    {
        private readonly NmeaDevice card;
        private readonly ThreadedLogger logger;

        public string portNum { get; }

        public PpsCard(string port, string logFilePath, string rawFilePath)
        {
            portNum = port;

            logger = new ThreadedLogger(logFilePath, "PpsCardLogger");
            logger.Start();

            logger.LogLine("time(UTC)\tdelay\taverage (N=90)\taverage (N=500)\taverage (N=1000)\ttemperature");

            card = new NmeaDevice(new SerialPort("COM" + port, 115200, Parity.None, 8, StopBits.One), rawFilePath, "PpsCard");
            card.MessageReceived += NmeaMessageReceived;
            card.OpenPort();
            card.startLogging();
        }

        private void NmeaMessageReceived(object sender_, EventArgs args_)
        {
            DateTime time = DateTime.UtcNow;

            NmeaDevice device = (NmeaDevice)sender_;
            NmeaMessage message_ = ((NmeaMessageReceivedEventArgs)args_).Message;

            if (message_.MessageType == "GPINF")
            {
                PpsInfo message = (PpsInfo)message_;
                logger.LogLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}",
                               time.ToString("yyyy/MM/dd HH:mm:ss"),
                               message.delay,
                               message.average_90,
                               message.average_500,
                               message.average_1000,
                               message.temperature
                               );
            }
        }

        public void Close()
        {
            card.MessageReceived -= NmeaMessageReceived;
            logger.Close();
            card.Close();
        }

        public void Pause(TimeSpan time)
        {
            logger.Pause(time);
        }

        public void ReStart()
        {
            logger.ReStart();
        }
    }
}
