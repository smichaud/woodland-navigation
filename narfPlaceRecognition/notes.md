# Just some notes to run the software

- Put the dataset in a single folder
- Format == PCD
- It is better to put a far range PCD file `(e.g scan_001.pcd with scan_001_far.pcd)`
- To get the SLAM, you need to add text files: `scan_001_info.dat`
    - `Odometry: 0 0 0 1.58 0.8 1.58`
    - `x y z roll pitch yaw`
- Create a single file for the dataset :
```
angularResolution 1.0
coodinateFrame 1
maxDistanceToConsiderScansOverlapping 3.0
shortName MyCoolDataset
```
- Check the created /build/bin/currentPlaceRecognitionParameters.ini file

- Steps:
1  Dataset->Recalculate and save range images
2. Dataset->Create dataset statistics.
2. Dataset->Recalculate and save all.
3. Dataset->Calculate and save confusion matrix.
4. Dataset->Do incremental scan matching
5. Dataset->Create and optimize graph (robust)
6. View->Show SLAM map
7. Database->Save know poses
