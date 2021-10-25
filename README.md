# KalmanFilter

### 1. Kalman Filter Algorithm

pipeline

1. inital value setting

     <img src="https://latex.codecogs.com/png.image?\dpi{130}&space;\bg_white&space;\hat{x}_{0},&space;P_{0}" title="\bg_white \hat{x}_{0}, P_{0}" />
2. 추정값과 오차 공분산 예측

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\left\{\begin{matrix}\hat{x}_{k}^{-}&space;=&space;A&space;\hat{x}_{k-1}&space;\triangleright1&space;\\P_{k}^{-}&space;=&space;A&space;P_{k-1}A^{T}&space;&plus;&space;Q&space;\triangleright2\end{matrix}\right." title="\bg_white \left\{\begin{matrix}\hat{x}_{k}^{-} = A \hat{x}_{k-1} \triangleright1 \\P_{k}^{-} = A P_{k-1}A^{T} + Q \triangleright2\end{matrix}\right." />
     
3. 칼만 이득 계산

    <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;K_{k}&space;=&space;P_{k}^{-}H^{T}(HP_{k}^{-}H^{T}&plus;R)^{-1}" title="\bg_white K_{k} = P_{k}^{-}H^{T}(HP_{k}^{-}H^{T}+R)^{-1}" />
    
4. 추정값 계산

    <img src="https://latex.codecogs.com/png.image?\dpi{130}&space;\bg_white&space;\hat{x}_{k}&space;=&space;\hat{x}_{k}^{-}&plus;K_{k}(z_{k}-H\hat{x}_{k}^{-})" title="\bg_white \hat{x}_{k} = \hat{x}_{k}^{-}+K_{k}(z_{k}-H\hat{x}_{k}^{-})" /> --> 측정값 <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;z_{k}" title="\bg_white z_{k}" />가 입력되면 추정값 <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}" title="\bg_white \hat{x}_{k}" />을 계산
    
5. 오차 공분산 계산

    <img src="https://latex.codecogs.com/png.image?\dpi{130}&space;\bg_white&space;P_{k}&space;=&space;P_{k}^{-}&space;-&space;K_{k}HP_{k}^{-}" title="\bg_white P_{k} = P_{k}^{-} - K_{k}HP_{k}^{-}" />
