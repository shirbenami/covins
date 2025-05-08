### Convert GT from EuRoC to TUM format

Use a Python script to:

1. Select: `timestamp, t_x, t_y, t_z, q_x, q_y, q_z, q_w`
2. Convert timestamp from nanoseconds to seconds
3. Sync with estimated trajectory timestamps (rounded to 3 decimal places)
4. Save as `mh123_gt_tum_synced.txt`

```
import pandas as pd
import numpy as np

df_gt = pd.read_csv("mh123_gt.csv", comment="#", header=None)

df_est = pd.read_csv("mh123_est_gba.csv", sep=' ', header=None)

df_gt[0] = pd.to_numeric(df_gt[0], errors='coerce') / 1e9

df_tum = df_gt[[0,1,2,3,5,6,7,4]]  # note: qw הולך בסוף

df_tum["rounded_time"] = df_tum[0].round(3)
df_est["rounded_time"] = df_est[0].round(3)

df_synced = df_tum[df_tum["rounded_time"].isin(df_est["rounded_time"])].drop(columns=["rounded_time"])

df_synced.to_csv("mh123_gt_tum_synced.txt", sep=' ', header=False, index=False)
```

#### After syncing GT and converting to TUM:

#### 🔹 without GBA:
 ```evo_ape tum mh123_gt_tum_synced.txt mh123_est.csv --align --correct_scale -p --plot_mode xz```

 | Metric | Value (m) |
| ------ | --------- |
| Max    | 0.120097  |
| Mean   | 0.048176  |
| Median | 0.042595  |
| Min    | 0.019948  |
| RMSE   | 0.053570  |
| SSE    | 0.083221  |
| Std    | 0.023425  |

   ![image](https://github.com/user-attachments/assets/fb5f7cd2-2fae-427c-86b8-ff6917f62507)
   ![image](https://github.com/user-attachments/assets/3d299a04-9363-45f1-ad2d-e4c30f7de4ff)
The spatial APE plots show that most of the estimated trajectory maintains low error (2–7 cm), especially in the central segments. However, local spikes in error are visible at trajectory edges, often due to abrupt motion or visually ambiguous regions. The time-based APE graph confirms this consistency, with the mean and RMSE around 5–6 cm and only a few short-lived deviations. These results suggest good global alignment, but with room for improvement in handling challenging regions (e.g., by enhancing visual tracking or incorporating loop closures).



#### 🔹 with GBA
```evo_ape tum mh123_gt_tum_synced.txt mh123_est_gba.csv --align --correct_scale -p --plot_mode xz```

| Metric | Value (m) |
| ------ | --------- |
| Max    | 0.090646  |
| Mean   | 0.016981  |
| Median | 0.014023  |
| Min    | 0.002275  |
| RMSE   | 0.020107  |
| SSE    | 0.286655  |
| Std    | 0.010768  |

![image](https://github.com/user-attachments/assets/c5a355c1-f416-489c-8710-737935a83c4e)
![image](https://github.com/user-attachments/assets/dd38fcd3-088d-4fff-a896-27dc0cb47e77)


---
