# Steps for 

- Generating navigation data from bag and 
- Manually sending the data to /jmc_auto/navigation topic 

### Step 1: In dev docker, extract path data from bags

```
dev_docker:/jmcauto$cd /modules/tools/navigator 
dev_docker:/jmcauto/modules/tools/navigator$python extractor.py path-to-bags/*.bag 
```

A path file will be generated in  

```
dev_docker:/jmcauto/modules/tools/navigator$ 
```

With format of  

```
path_[first_bag_name].bag.txt 
```



### Step2: (Optional) Verify the extracted path is correct   

dev_docker:/jmcauto/modules/tools/navigator$python viewer_raw.py path_[bag_name].bag.txt 

### Step3: Smooth the path  

```
dev_docker:/jmcauto/modules/tools/navigator$./smooth.sh /jmcauto/modules/tools/navigator/path_[first_bag_name].bag.txt 200
```

200 is the parameter for smooth length. If the smooth is failed, try to change this parameter to make the smooth pass. The prefered number is between 150 and 200. 

A smoothed data file, path_[first_bag_name].bag.txt.smoothed, is generated under folder   

```
dev_docker:/jmcauto/modules/tools/navigator$
```

### Step4: (Optional) Verify the smoothed data

```
dev_docker:/jmcauto/modules/tools/navigator$ python viewer_smooth.py path[first_bag_name].bag.txt path[first_bag_name].bag.txt.smoothed 
```



### Step5: Send /jmc_auto/navigation topic 

Run follow command to send /jmc_auto/navigation  data 

```
dev_docker:/jmcauto/modules/tools/navigator$python navigator.py path_[first_bag_name].bag.txt.smoothed
```
