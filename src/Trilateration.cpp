#include "kalman_filter/Trilateration.h"


using Trila::Trilateration;
using Trila::StationData;
using Trila::PosDataVec3d;
using Trila::PosData3d;

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

Trilateration::Trilateration(std::string file_name, ros::NodeHandle n) {
    
    ReadPosi(file_name);
    _posi_pub = n.advertise<geometry_msgs::Point>("global_position", 1);

}

void Trilateration::PosiCalcu(/*const renov_localization::uwb_info &range*/) {

    Eigen::Vector3d location;
    int n, rows;
    int count = 0;

    if (_uwb_input.size() < 3) {
        std::cout << "The quantity of stations is not enough" << std::endl;
    }

    // "n" refers to quantity of UWB stations
    // "rows" refers to the quantity of equations
    n = _uwb_input.size();
    rows = n*(n-1) / 2;
    Eigen::MatrixXd m(rows, 3);
    Eigen::VectorXd result(rows);

    for (size_t i = 0; i < n; i++) {

        double x1, x2, y1, y2, z1, z2;
        double d1, d2;
        x1 = _uwb_input[i]._pos(0);
        y1 = _uwb_input[i]._pos(1);
        z1 = _uwb_input[i]._pos(2);
        d1 = _uwb_input[i]._dis;

        for (size_t j = i+1; j < n; j++) {

            x2 = _uwb_input[j]._pos(0);
            y2 = _uwb_input[j]._pos(1);
            z2 = _uwb_input[j]._pos(2);
            d2 = _uwb_input[j]._dis;

            m(count, 0) = x1 - x2;
            m(count, 1) = y1 - y2;
	        m(count, 2) = z1 - z2;
            result(count) = (pow(x1, 2) - pow(x2, 2) +
                       	     pow(y1, 2) - pow(y2, 2) +
                             pow(z1, 2) - pow(z2, 2) +
                             pow(d2, 2) - pow(d1, 2)) / 2;
	        count++;
        }    
    }	
    // std::cout << m << std::endl << std::endl;

    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Eigen::MatrixXd U = svd.matrixU();
    // Eigen::MatrixXd V = svd.matrixV();
    // Eigen::MatrixXd Vt = V.transpose();
    
    // std::cout << "The singular values are : " << svd.singularValues() << std::endl
	//       << U << std::endl << std::endl
  	//       << V << std::endl << std::endl;

    location = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(result);

    geometry_msgs::Point coordinates;
    coordinates.x = location(0);
    coordinates.y = location(1);
    coordinates.z = location(2);

    _posi_pub.publish(coordinates);

    std::cout << "The location is [ "
              << "x : " << coordinates.x << "  "
              << "y : " << coordinates.y << "  " 
              << "z : " << coordinates.z << " ]" << std::endl;

}

void Trilateration::ReadPosi(std::string file_name) {
    
    std::fstream fins(file_name, std::fstream::in);
    if (fins.is_open()) {
        std::string line;
        while (getline(fins, line)) {
            std::stringstream ss(line);
            double x, y, z;
            ss >> x >> y >> z;
            PosData3d uwb_in(Eigen::Vector3d(x, y, z), 0);
            _uwb_input.push_back(uwb_in);
        }        
        std::cout << "UWB positions got" << std::endl;
    } else {
        std::cerr << "I don't have UWB position" << std::endl;
    }
    fins.close();

}
