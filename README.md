# Mapping-of-Nearshore-Bathymetry-Based-on-UAVs-Video-Stitching

1.Prepare  
&ensp;&ensp;You may use the external libraries  
&ensp;&ensp;CVX: http://cvxr.com/cvx/  
&ensp;&ensp;vlfeat: http://www.vlfeat.org/  
&ensp;&ensp;then downsample you videos. For example, the downsample frequency is 2 hz, you video fps is 30 frames per second, that is to say you should select 2 frames per 30 frames in a second, just selecting the 1st frame and 16th frame in the first second is OK, and 31st, 46th,  61st frame successively.
2.SingleFlight  
&ensp;&ensp;This code is for a single UAV bathymetry  
3.MultipleFlight  
&ensp;&ensp;This code is for two UAV bathymetry, which uses Video Stitching algorithm  
&ensp;&ensp;Some source code are also provided by Su Tan, the original copy can be downloaded from this webpage https://github.com/SuTanTank/VideoStitchingViaShakinessRemoving  
4.bathymetry method  
&ensp;&ensp;Including timeCor and cBathy algorithm.
