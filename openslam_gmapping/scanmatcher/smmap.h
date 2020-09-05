#ifndef SMMAP_H
#define SMMAP_H
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#define SIGHT_INC 1
#define LOGFREE -1  // for occupancy grid map
#define LOGOCC 2    // for occupancy grid map
namespace GMapping
{
    struct PointAccumulator
    {
            typedef point<float> FloatPoint;
            // pMapTSDF(-1), for TSDF mapping algorithm, default -1 means out of the sensor's detection area
            // pMapHits(0),pMapMisses(0), for counting model method
            // pLog(-1), for occupancy grid map, default -1 means out of the sensor's detection area
            PointAccumulator(): acc(0,0), n(0), visits(0),pMapTSDF(-1),pMapHits(0),pMapMisses(0),pLog(-1){}
            PointAccumulator(int i): acc(0,0), n(0), visits(0),pMapTSDF(-1),pMapHits(0),pMapMisses(0),pLog(25){assert(i==-1);}
            inline void update(bool value, const Point& p=Point(0,0));
            inline Point mean() const {return 1./n*Point(acc.x, acc.y);}
            inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
            inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits;pMapTSDF+=p.pMapTSDF; pMapHits+=p.pMapHits;pMapMisses+=p.pMapMisses;pLog+=p.pLog;}
            static const PointAccumulator& Unknown();
            static PointAccumulator* unknown_ptr;
            FloatPoint acc;
            int n, visits;
            inline double entropy() const;
            float pMapTSDF;
            unsigned long pMapHits, pMapMisses;
            int pLog;
    };

    void PointAccumulator::update(bool value, const Point& p)
    {
        if (value) //hit
        {
                acc.x+= static_cast<float>(p.x);
                acc.y+= static_cast<float>(p.y);
                n++;
                visits+=SIGHT_INC;

                // when hit, update the likelyhood in TSDF map
                pMapTSDF=((visits-1)*pMapTSDF+1)/(visits);

                // hit count + 1, for counting model method
                pMapHits++;

                // when hit, update the log probabilistic in occupancy grid map
                pLog= (pLog+LOGOCC) > 100 ? 100:(pLog+LOGOCC);
        }
        else //miss
        {
            visits++;

            // when miss, update the likelyhood in TSDF map
            pMapTSDF=((visits-1)*pMapTSDF)/(visits);

            // hit count + 1, for counting model method
            pMapMisses++;

            // when miss, update the log probabilistic in occupancy grid map
            pLog= (pLog+LOGFREE) < 0 ? 0:(pLog+LOGFREE);
        }
    }

    double PointAccumulator::entropy() const
    {
            if (!visits)
                    return -log(.5);
            if (n==visits || n==0) 
                    return 0;
            double x=(double)n*SIGHT_INC/(double)visits;
            return -( x*log(x)+ (1-x)*log(1-x) );
    }
    typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;

};

#endif 
