class Robot {
public:
    Robot();
    void move(double dx, double dy);
    double getX() const;
    double getY() const;

private:
    double x_;
    double y_;
};
