-------created by xiaotian at 11/07/20-------

Prject: Efmi-Omni-Sinusoid

-------
main program: ./test/main_transflow.cpp

program arguments:
dataset/PanoraMIS_Sequence6/000022_0290_0162_0526.png
dataset/PanoraMIS_Sequence6/000023_0307_0173_0535.png
(take images 22 23 as example, because the motion is mainly translation, only with a small yaw.)

working directory: Efmi-Omni-Sinusoid

in case some unexpected problems, you can run programs in my computer. And the password is 'marsmars'.
-------
./dataset: PanoraMIS Sequence 6 (figure-of-eight trajectory) 
	   https://home.mis.u-picardie.fr/~panoramis/

-------
./results: /22_23/new_results/du(v)_data.txt
	   (the main result, stich all translines projected in u/v direction.)
	    --------> projected transline at one pixel position, with du(v) increasing
	    |
	    |
	    |
	    V
	    with pixel position increasing

	   /22_23/original_have
	   some results the former programs output.

-------
./plot: MATLAB programs for plotting. 
	details can be found in ./plot/README.MD


---------------------------------------------
11/07 update

Results:
The results is not good. 
However, by observing new_results/dus_wave_max.jpg and original_have/dxs.jpg, we can find some similiarities.
Notice that in many pixels, have no du/dv, because the cos/sin(trans_angle) is very closed to zero. For example: from sub_image 7 in du_data.

Potential Problems:
1. The method to sample the sector with max energy.
    1) The sample frequency should be larger (especially when it close to center, or zero point of sinusoid curve).
    2) max_element and normalizetion using energy, could improve it?
2. The choose of translation angle.
    For now, I choose the angle of sector with max energy in the results[0]'s phase shift diagram (after rerotate and rescale) as the trans angle.
    But maybe there are better choose, like atan(trans_y,trans_x). And these two angles always differ 0.1-0.2 rad, which might be a huge error.
3. The dataset.
    Maybe we could create/find clearer images with more distinct seperate depths...

---------------------------------------------
11/14 update

1. change the resoltion of pano image from 241 to 250. Get a little improvement in results.
2. A huge BUG... = -> +=, better.
3. PROBLEMS: sometimes  maxAngle differ pi from atan(y,x)..

---------------------------------------------
11/15 update
TODO: 1. DEBUG: +-180 degree.
      2. Dataset: 



