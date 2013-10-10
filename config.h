/**
 * Copyright 2013 CPI Group, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CONFIG_H
#define	CONFIG_H

// output
#define OUTPUTSCREEN        1
#define OUTPUTJSONFILE      "./boxDimenstions.json"
#define OUTPUTJSONCOUNT     10          // how many records keep in JSON file

#define SIDECOMPAREINCH     0.5f

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

