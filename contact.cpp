////--------- PUT IN SETUP
  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();
///----------------------

class Accelerometer : public Zumo32U4IMU
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } acc_data_xy;

  public:
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
  private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;
};

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg());
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (acc.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

void on_contact_made()
{
  in_contact = true;
}

// reset forward speed
void on_contact_lost()
{
  in_contact = false;
}
