class Timer
{
  public:
    Timer(unsigned int Millisec);
    bool checkT();
    int t;
  private:
    unsigned long _currentTime;
    unsigned long _previousTime;
    double _elapsedTime;
    unsigned int _ms;
};
