#ifndef EL_FORCE_HPP
#define EL_FORCE_HPP

#include <vector>

class ElForce
{
private:
    double k_, ownCharge_, enemyCharge_, targetCharge_;
    std::vector<double> target_;
    std::vector<std::vector<double>> enemies_;

public:
    ElForce(double k, double ownCharge, double enemyCharge, double targetCharge);

    void setTarget(std::vector<double> targetPos);
    void addEnemy(std::vector<double> enemyPos);
    void clearEnemies();
    double calculateDistance(std::vector<double> p1, std::vector<double> p2);
    double calculateForce(std::vector<double> dronePos);
};

#endif