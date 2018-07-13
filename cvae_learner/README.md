- Learning Model  

# Running the Model  
- python train_z6.py (set data_directory and graph in line 127/128)  
- python test_z6.py --type SP/RF/RF_Robust/Halton --modelfile checkpoints_xyz/model.ckpt/None  
Note :  
 can use modelfile as None if testing Halton Samples  
 default test data is in file Results/conditions.txt