These scripts were simple experiments with basic image processing techniques. 

- rgb_channels.py separates image channels (I was trying to get an intuition for how well an independent channel could be to identify plants)
- color_similarity.py allows you to set a target color to compare every pixel against
- sam.py creates masks using SAM, but it takes on the order of 30-60 sec to process 1 image on my CPU
    - you can download the desired SAM model checkpoint here: https://github.com/facebookresearch/segment-anything?tab=readme-ov-file#model-checkpoints
