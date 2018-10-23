using System;
using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;
using System.IO;

namespace CustumLoggers
{

    class ThreadedLogger
    {
        Thread thread;
        string path;
        string name;
        ConcurrentQueue<string> queue;

        public TimeSpan RetryDelay;
        public TimeSpan SleepTime;


        public ThreadedLogger(string path, string name)
        {
            ThreadLoggerCons(path, name, new TimeSpan(0, 0, 0, 2), new TimeSpan(0, 0, 0, 0, 500));
        }
        public ThreadedLogger(string path, string name, TimeSpan retryDelay, TimeSpan sleepTime)
        {
            ThreadLoggerCons(path, name, retryDelay, sleepTime);
        }
        private void ThreadLoggerCons(string path, string name, TimeSpan retryDelay, TimeSpan sleepTime)
        {
            RetryDelay = retryDelay;
            SleepTime = sleepTime;

            this.path = Path.GetFullPath(path);
            if (!Path.IsPathRooted(path))
                throw new Exception("Logger: realative path");

            File.Create(this.path);

            this.name = name;
            queue = new ConcurrentQueue<string>();
            thread = new Thread(new ThreadStart(logger));
            thread.Name = name;
            thread.IsBackground = true;
        }

        public void Start()
        {
            if ((thread.ThreadState & System.Threading.ThreadState.Unstarted) == System.Threading.ThreadState.Unstarted)
                thread.Start();
            else
                throw new Exception("Logger: double start");
        }

        public void Log(string snippet)
        {
            queue.Enqueue(snippet);
        }
        public void LogLine(string snippet)
        {
            queue.Enqueue(snippet + "\r\n");
        }

        private void logger()
        {
            //clean the queue
            string trash;
            while (!queue.IsEmpty)
                while (!queue.TryDequeue(out trash))
                    Thread.Yield();

            while (true)
            {
                while (!queue.IsEmpty)
                {
                    try
                    {
                        using (StreamWriter write = new StreamWriter(path, true))
                        {
                            while (!queue.IsEmpty)
                            {
                                string log;
                                while (!queue.TryPeek(out log))
                                    Thread.Yield();

                                write.Write(log);

                                while (!queue.TryDequeue(out log))
                                    Thread.Yield();
                            }
                        }
                    }
                    catch (Exception)
                    {
                        Thread.Sleep((int)Math.Round(RetryDelay.TotalMilliseconds));
                    }
                }

                Thread.Sleep((int)Math.Round(SleepTime.TotalMilliseconds));
            }
        }
    }
}
