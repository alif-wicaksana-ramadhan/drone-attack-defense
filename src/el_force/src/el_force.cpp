#include <cmath>
#include "el_force/el_force.hpp"

ElForce::ElForce(double k, double ownCharge, double enemyCharge, double targetCharge) : k_(k), ownCharge_(ownCharge), enemyCharge_(enemyCharge), targetCharge_(targetCharge)
{
    target_ = {0.0, 0.0, 0.0};
}

void ElForce::setTarget(std::vector<double> target_pos)
{
    target_ = target_pos;
}

void ElForce::addEnemy(std::vector<double> enemyPos)
{
    enemies_.push_back(enemyPos);
}

void ElForce::clearEnemies()
{
    enemies_.clear();
}

double ElForce::calculateDistance(std::vector<double> p1, std::vector<double> p2)
{
    double sum = 0.0;
    for (int i = 0; i < (int)p1.size(); ++i)
    {
        sum += std::pow(p2[i] - p1[i], 2);
    }
    return std::sqrt(sum);
}

double ElForce::calculateForce(std::vector<double> dronePos)
{
    double d = calculateDistance(dronePos, target_);
    return k_ * ownCharge_ * targetCharge_ / (d * d);
}