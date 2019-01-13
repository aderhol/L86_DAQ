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
        PreAll writer;
        string name;
        ConcurrentQueue<string> queue;

        public TimeSpan RetryDelay;
        public TimeSpan SleepTime;


        public ThreadedLogger(string path, string name)
        {
            ThreadLoggerCons(path, name, new TimeSpan(0, 0, 0, 2), new TimeSpan(0, 0, 0, 0, 500), 1024 * 1024 * 10);
        }
        public ThreadedLogger(string path, string name, int fragmantSize)
        {
            ThreadLoggerCons(path, name, new TimeSpan(0, 0, 0, 2), new TimeSpan(0, 0, 0, 0, 500), fragmantSize);
        }
        public ThreadedLogger(string path, string name, TimeSpan retryDelay, TimeSpan sleepTime)
        {
            ThreadLoggerCons(path, name, retryDelay, sleepTime, 1024 * 1024 * 10);
        }
        public ThreadedLogger(string path, string name, TimeSpan retryDelay, TimeSpan sleepTime, int fragmantSize)
        {
            ThreadLoggerCons(path, name, retryDelay, sleepTime, fragmantSize);
        }
        private void ThreadLoggerCons(string path, string name, TimeSpan retryDelay, TimeSpan sleepTime, int fragmantSize)
        {
            RetryDelay = retryDelay;
            SleepTime = sleepTime;

            this.path = Path.GetFullPath(path);
            if (!Path.IsPathRooted(path))
                throw new Exception("Logger: realative path");

            writer = new PreAll(path, fragmantSize, 0.01);

            this.name = name;
            queue = new ConcurrentQueue<string>();
            thread = new Thread(new ThreadStart(logger));
            thread.Name = name;
            thread.IsBackground = true;
        }

        public void Start()
        {
            if ((thread.ThreadState & System.Threading.ThreadState.Unstarted) == System.Threading.ThreadState.Unstarted)
            {
                thread.Start();
                isStarted.WaitOne();    //doesn't return until the logging has started and the queue has been cleaned
            }
            else
                throw new Exception("Logger: double start");
        }

        public void Log(string snippet, params object[] args)
        {
            queue.Enqueue(String.Format(snippet, args));
        }
        public void LogLine(string snippet, params object[] args)
        {
            this.Log(snippet + "\r\n", args);
        }
        public void LogLine()
        {
            queue.Enqueue("\r\n");
        }

        ManualResetEvent isStarted = new ManualResetEvent(false);
        private void logger()
        {
            //clean the queue
            string trash;
            while (!queue.IsEmpty)
                while (!queue.TryDequeue(out trash))
                    Thread.Yield();
            isStarted.Set();

            while (!isCloseRequested_.IsCancellationRequested)
            {
                while (!queue.IsEmpty)
                {
                    try
                    {
                        while (!queue.IsEmpty)
                        {
                            string log;
                            while (!queue.TryPeek(out log))
                                Thread.Yield();

                            writer.Write(log);

                            while (!queue.TryDequeue(out log))
                                Thread.Yield();
                        }

                    }
                    catch (Exception)
                    {
                        Thread.Sleep((int)Math.Round(RetryDelay.TotalMilliseconds));
                    }
                }

                Thread.Sleep((int)Math.Round(SleepTime.TotalMilliseconds));
            }
            stopped.Set();
        }

        private CancellationTokenSource isCloseRequested_ = new CancellationTokenSource();
        private ManualResetEvent stopped = new ManualResetEvent(false);
        public void Close()
        {
            isCloseRequested_.Cancel();
            stopped.WaitOne();
            writer.Close();
        }
    }

    class PreAll
    {
        private FileStream fs_;
        private StreamWriter writer_;
        private int fragmantSize_;
        private double thresholdRatio_;

        public PreAll(string path, int fragmantSize, double thresholdRatio)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            fs_ = File.Open(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read);
            writer_ = new StreamWriter(fs_);
            fragmantSize_ = fragmantSize;
            thresholdRatio_ = thresholdRatio;
        }

        public void Write(string str)
        {
            if (fs_.Length - fs_.Position < fragmantSize_ * thresholdRatio_)
                fs_.SetLength(fs_.Length + fragmantSize_);

            if (fs_.Position > 3)
                fs_.Position -= 3;
            writer_.Write(str);
            writer_.Write(@"@~@");
            writer_.Flush();
        }

        public void WriteLine(string str)
        {
            this.Write(str + "\r\n");
        }

        public void Close()
        {
            if (fs_.Position > 3)
                fs_.Position -= 3;
            fs_.SetLength(fs_.Position);
            writer_.Close();
        }
    }
}

