#ifndef CONFIG_H
#define	CONFIG_H

// constants
#define METERSTOINCH        39.37

#define LINEPOINTSTEP       0.0001      // drawing line distance

#define MAXDISTANCE         1.4f        // set workign distance

#define BACKGROUNDCAPTURE   100         // pages for background

#define LEAFSIZE            0.005f      // cm for mesh
#define RANSACDISTANCE      0.0045      // cm for ransac.setDistanceThreshold

#define CLUSTERTOLETANCE    0.01        // cm 
#define MINClUSTERSIZE      100         // min size in point
#define MAXCLUSTERSIZE      25000       // max size in point

#define RGBCOLOR_GRAY       rgb(189, 189, 189)
#define RGBCOLOR_LIGHTGRAY  rgb(214, 214, 214)
#define RGBCOLOR_WHITE      rgb(255, 255, 255)
#define RGBCOLOR_RED        rgb(245,  20,  20)
#define RGBCOLOR_GREEN      rgb( 20, 245,  20)
#define RGBCOLOR_BLUE       rgb( 20,  20, 245)

#endif	/* CONFIG_H */

