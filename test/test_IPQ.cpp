//
// Created by yutou on 3/2/22.
//

#include <bits/stdc++.h>
#include <ESDFMap.h>
#include <cstdlib>
#include <utility>
#include <IndexedPriorityQueue.h>

#include <parameters.h>

using namespace std;
//namespace fiesta {

typedef fiesta::IndexedPriorityQueue<Eigen::Vector3i, fiesta::IPQ_element, fiesta::cmp_element, fiesta::MatrixHash<Eigen::Vector3i>> Indexed_voxel_queue;

void display(const Indexed_voxel_queue& IPQ){
    Indexed_voxel_queue ipq = IPQ;
    while (!ipq.empty()){
        pair<Eigen::Vector3i, fiesta::IPQ_element> temp = ipq.top();
        ipq.pop();
        Eigen::Vector3i point = temp.first;
        cout << "( [" << point.x() << ", " << point.y() << ", " << point.z() << "] ";
        cout << temp.second.distance_ << ")" << endl;
    }
}

void get_batch(Indexed_voxel_queue& IPQ, vector<fiesta::IPQ_element>& batch_vector){
    batch_vector.clear();
    double batch_sqr_dist = IPQ.top().second.squared_distance_;
    while(!IPQ.empty() && abs(batch_sqr_dist - IPQ.top().second.squared_distance_) < 1e-4){
        batch_vector.push_back(IPQ.top().second);
        IPQ.pop();
    }
}

int main(int argc, char **argv) {

    Eigen::Vector3i obstacle(0, 0, 0);
    int N = fiesta::num_dirs_;

    // values to be searched later
    Eigen::Vector3i target_point;
    double target_sqr_dist;

    fiesta::IndexedPriorityQueue<Eigen::Vector3i, fiesta::IPQ_element, fiesta::cmp_element, fiesta::MatrixHash<Eigen::Vector3i>> IPQ;

    for (int i = 0; i < N; i++) {
        Eigen::Vector3i point = obstacle + fiesta::dirs_[i];
        fiesta::IPQ_element tmp_element(point, point.cast<float>().norm(), obstacle);
        IPQ.push(point, tmp_element);

        /* */
        if (i==3){
            target_point = point;
            target_sqr_dist = tmp_element.squared_distance_;
        }
    }

    cout << "In ascending order:" << endl;
    display(IPQ);

    cout << "Processing in batches: " << endl;
    vector<fiesta::IPQ_element> batch_vector;
    while(!IPQ.empty()){
        get_batch(IPQ, batch_vector);
        for (auto element : batch_vector){
            cout << "( [" << element.point_.x() << ", " << element.point_.y() << ", " << element.point_.z() << "] ";
            cout << element.distance_ << ")" << endl;
        }
        cout << endl;
    }

    return 0;
}

//int main(int argc, char** argv) {
//    fiesta::IndexedPriorityQueue<int, int> IPQ;
//
//    vector<int> A = {2, 7, 26, 25, 19, 17, 1, 90, 3, 36};
//    // Insertion
//    for (int i = 0; i < A.size(); i++) {
//        IPQ.push(i, A[i]);
//    }
//
//    long long size = IPQ.size();
//    for (int i = 0; i < size; i++ ){
//        IPQ.pop();
//    }
//
//}

//    } // namespace fiesta