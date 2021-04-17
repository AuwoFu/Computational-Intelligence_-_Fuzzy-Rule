# Computational-Intelligence_Fuzzy-Rule

使用模糊規則設計自走車成功走到終點

# 操作
- 顯示：
  - 右側:顯示地圖與行進動畫；
  - 左側上方:文字欄位依序為中央探測器偵測距離，左側偵測距離，右側偵測距離，車輛當前方向，偵測後接下來要轉的角度；
  - 左下方:狀態顯示
- 按鈕：
  - Start：開始/從頭開始
  - Pause：暫停，再按一次為繼續
  - Choose Map:選擇地圖，會根據地圖格式判斷正確與否，關閉選擇視窗則維持當前地圖
  - Choose Record：選擇路徑檔，檔案格式採用6D，路徑檔選擇後須選擇採用的地圖，若關閉視為採用當前地圖

# 模糊規則
使用的測試地圖無岔路，無須分辨向左轉或是向右轉，因此只需判斷是否轉彎即可，能轉彎的前提是左方或右方不為牆，也就使左右兩邊的探測長度不相同，最終的模糊規則如下：

- 若兩邊相距不大，即代表還未到轉彎處，因此保持既有方向
- 若兩邊相距較大，則根據距離差決定轉幾度

方程式如下，l為左探測距離，r為右探測距離，相距不大定義為小於7，max⁡difference代表l與r相距多大時必須立刻轉動最大角度，設定為15。
<img width="273" alt="3" src="https://user-images.githubusercontent.com/59794062/115122159-0ed57a80-9fe9-11eb-9c72-6033225975e6.PNG">


![1](https://user-images.githubusercontent.com/59794062/115122198-493f1780-9fe9-11eb-9cc3-fd5fc7090524.jpg =188x100)
![2](https://user-images.githubusercontent.com/59794062/115122200-4b08db00-9fe9-11eb-9f44-04a657e893e2.jpg =156x100)


