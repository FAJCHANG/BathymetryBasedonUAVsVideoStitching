# Mapping of Nearshore Bathymetry Based on UAVs Video Stitching

**1.Prepare**  
&ensp;&ensp;Run environment: MatlabR2019b, python3  
&ensp;&ensp;You may use the external libraries  
&ensp;&ensp;CVX: http://cvxr.com/cvx/  
&ensp;&ensp;vlfeat: http://www.vlfeat.org/  
&ensp;&ensp;then downsample you videos. For example, the downsample frequency is 2 hz, you video fps is 30 frames per second, that is to say you should select 2 frames per 30 frames in a second, just selecting the 1st frame and 16th frame in the first second is OK, and 31st, 46th,  61st frame successively.  
  
**2.SingleFlight**  
&ensp;&ensp;This code is for a single UAV or video bathymetry.Program entry ``main.m``. It contains  
&ensp;&ensp;1. DownSample by python;  
&ensp;&ensp;2. Estimating camera motion;  
&ensp;&ensp;3. Calculate camera extrinsics parameters;  
&ensp;&ensp;4. Orthomorphic transformation;  
&ensp;&ensp;For specific operations and parameters, see the corresponding readme and comments between codes.  
  
**3.MultipleFlight**  
&ensp;&ensp;This code is for two UAVs or videos bathymetry, which uses Video Stitching algorithm. Program entry ``main.m``. It contains  
&ensp;&ensp;1. Tracking feature points;  
&ensp;&ensp;2. Background identification;  
&ensp;&ensp;3. Video Stitching;  
&ensp;&ensp;4. Estimating camera motion;  
&ensp;&ensp;5. Calculate camera extrinsics parameters;  
&ensp;&ensp;6. Orthomorphic transformation;  
&ensp;&ensp;But first of all, you have to down-sample the two videos and place the pictures to ``./dataset/left/`` and ``./dataset/right/``.  
&ensp;&ensp;*Note: Please calibrate your camera's intrinsics parameters in advance. And place it to ``./Stitching/neededData/``.  
&ensp;&ensp;For specific operations and parameters, see the corresponding readme and comments between codes.  
&ensp;&ensp;Some source code are also provided by Su Tan,  
the original copy can be downloaded from this webpage https://github.com/SuTanTank/VideoStitchingViaShakinessRemoving  
  
**4.Bathymetry method**  
&ensp;&ensp;Including timeCor and cBathy algorithm.  
&ensp;&ensp;When you finished SingleFlight or MultipleFlight mission, save the filter image, orthoimage, camera extrinsics files, then set parameters ``bathyParams.m`` file, (cite a case timeCor), run ``mainMakeTimeStack.m`` -> ``mainV2.m``.
