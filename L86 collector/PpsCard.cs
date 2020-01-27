﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CustumLoggers;
using NMEA_Parser;
using SerialPort_Win32;

namespace PpsCardDelivery
{
    class PpsCard
    {
        private readonly NmeaDevice card;
        private readonly ThreadedLogger logger;

        public string InputResourceLocator { get; }

        public PpsCard(string inputResourceLocator, string logFilePath, string rawFilePath, string errFilePath)
        {
            InputResourceLocator = inputResourceLocator;

            logger = new ThreadedLogger(logFilePath, "PpsCardLogger");
            logger.Start();

            logger.LogLine("time(UTC)\tdelay\taverage (N=90)\taverage (N=500)\taverage (N=1000)\ttemperature");


            if (int.TryParse(inputResourceLocator, out int comPortNumber))
            {
                card = new NmeaDevice(new ComPort(comPortNumber, 115200, Parity.NOPARITY, 8, StopBits.ONESTOPBIT), rawFilePath, errFilePath, "PpsCard");
            }
            else
            {
                card = new NmeaDevice(inputResourceLocator, rawFilePath, errFilePath, "PpsCard");
            }

            card.MessageReceived += NmeaMessageReceived;
            card.OpenPort();
            card.startLogging();
            card.ResetInputStream();
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

        public bool Paused
        {
            get
            {
                return logger.Paused;
            }
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
