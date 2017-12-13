EM1d_3Gauss takes 3 means, 3 stnd devs and takes samples from them. From those samples 3 gaussians are found
using GMM.

EM_NxD does the same with D dimensions and N gaussians

GMMLearning takes buoy pixel info for each buoy and uses GMM to create gaussians. Histograms of each channel
for each buoy are made, they were inspected to decide number of gaussians to make.
GMM_detectBuoy uses this info to detect buoys for the frames of the video.

GMMLearning must be run first, GMM_detectBuoy does not clear any variables, it is run after GMMLearning.