#ifndef GRIDDATA_H
#define GRIDDATA_H

#include <cmath>
#include <iterator>

namespace SpatialGridMap {
    enum Occupancy { 
        FREE = 0, 
        OCCUPIED, 
        UNKNOWN 
    };

    // We dont want this data to waste space in every Data-instance
    // Therefore it is static global, not the best solution maybe
    static double DOUBLE_MERGE_EPS = (1e-4);

    struct GridMapData {
        Occupancy occupancy;
        double pdf;

        GridMapData(){};
        GridMapData(Occupancy o, double p) : occupancy(o), pdf(p) {};

        static void setMergeEps(double eps){
            DOUBLE_MERGE_EPS = eps;
        }

        // Equal if obs is same, and prob within threshold
        bool operator==(GridMapData & other){
            if(other.occupancy != occupancy) return false;
            return(fabs(other.pdf - pdf) < DOUBLE_MERGE_EPS);
        }

        // Sets this data to be a merge of n other data instances
        template <class Iterator>
        void merge(Iterator bloxels, int n){
            //obs-values are assumed to be equal, otherwise merging should not be done
            occupancy = bloxels->data.occupancy;

            //set prob as average of all values
            double tmp = 0;
            for(int i=0; i<n; i++){
                tmp += bloxels->data.pdf;
                bloxels++;
            }
            pdf = tmp / n;
        }
    };

    inline std::ostream& operator << (std::ostream& stream, const GridMapData & data)
    {
        stream << data.occupancy << " " << data.pdf << " ";
        return stream; 
    }

    inline std::istream& operator >> (std::istream& stream, GridMapData & data)
    {
        int tmp;
        stream >> tmp >> data.pdf;
        data.occupancy = (Occupancy)tmp;
        return stream; 
    }

}; // end of namespace

#endif
