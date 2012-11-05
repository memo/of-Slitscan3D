#pragma once

#include "ofMain.h"

namespace msa {
    
    //--------------------------------------------------------------
    //--------------------------------------------------------------
    //--------------------------------------------------------------
    template <typename T>   // the type of data stored in each quantum cell
    class SpaceT {
    public:
        
        //--------------------------------------------------------------
        SpaceT(ofVec3f numCells, ofVec3f bmin, ofVec3f bmax) {
            setNumCells(numCells);
            setBoundaries(bmin, bmax);
        }
        
        //--------------------------------------------------------------
        // get number of quantum cells on each axis
        ofVec3f getNumCells() {
            return numCells;
        }
        
        //--------------------------------------------------------------
        // set number of quantum cells on each axis
        void setNumCells(ofVec3f numCells) {
            this->numCells = numCells;
            
            data.resize(numCells.x * numCells.y * numCells.z);
        }
        
        //--------------------------------------------------------------
        // set physical world boundaries on each axis
        void setBoundaries(ofVec3f bmin, ofVec3f bmax) {
            boundaryMin = bmin;
            boundaryMax = bmax;
        }
        
        
        //--------------------------------------------------------------
        // get quantum index given a physical world position
        ofVec3f getIndexForPosition(ofVec3f p) {
            ofVec3f v;
            v.x = ofMap(p.x, boundaryMin.x, boundaryMax.x, 0, numCells.x-1, true);
            v.y = ofMap(p.y, boundaryMin.y, boundaryMax.y, 0, numCells.y-1, true);
            v.z = ofMap(p.z, boundaryMin.z, boundaryMax.z, 0, numCells.z-1, true);
            return v;
        }
        
        
        //--------------------------------------------------------------
        // get quantum data for given quantum index
        T& getDataAtIndex(ofVec3f index) {
            return getDataAtIndex(index.x, index.y, index.z);
        }
        
        
        //--------------------------------------------------------------
        // get quantum data for given quantum index
        T& getDataAtIndex(int i, int j, int k) {
            return data[k * numCells.x * numCells.y + j * numCells.x + i];
        }
        
    protected:
        ofVec3f numCells;
        ofVec3f boundaryMin, boundaryMax;
        vector<T> data;
        
    };
    
    
    
    //--------------------------------------------------------------
    //--------------------------------------------------------------
    //--------------------------------------------------------------
    template <typename T>
    class SpaceTime {
    public:
        
        //--------------------------------------------------------------
        // set maximum number of quantum time frames
        void setMaxFrames(int m) {
            maxFrames = m;
        }
        
        //--------------------------------------------------------------
        // get current number of quantum time frames
        int getNumFrames() {
            return spaces.size();
        }

        //--------------------------------------------------------------
        // get Space data for given quantum frame (0...numFrames-1)
        SpaceT<T>* getSpaceAtFrame(int f) {
            return spaces[f];
        }
        
        //--------------------------------------------------------------
        // get Space data for given quantum time (0...1)
        SpaceT<T>* getSpaceAtTime(float t) {
            return getSpaceAtFrame(floor(t * (spaces.size()-1)));
        }

        
        //--------------------------------------------------------------
        // insert a new Space data (to time==0)
        void addSpace(SpaceT<T>* space) {
            spaces.insert(spaces.begin(), space);
            while(spaces.size() > maxFrames) {
                delete spaces.back();
                spaces.pop_back();
            }
        }
        
        //--------------------------------------------------------------
        void clear() {
            for(int i=0; i<spaces.size(); i++) {
                delete spaces[i];
            }
            spaces.clear();
        }

        
    protected:
        vector< SpaceT<T>* > spaces;
        int maxFrames;
    };
}