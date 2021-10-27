# KalmanFilter

### - Kalman Filter : 선형시스템 

<p align="center">PipeLine

<p align="center"><img src="https://github.com/junhyukch7/Kalman-Filter/blob/main/pipeline.png" width="50%">


- 변수

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;z_{k}" title="\bg_white z_{k}" /> : 측정값

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}" title="\bg_white \hat{x}_{k}" /> : 추정값

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;P_{k}" title="\bg_white P_{k}" /> : <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}" title="\bg_white \hat{x}_{k}" />의 오차 공분산

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}^{-}" title="\bg_white \hat{x}_{k}^{-}" /> : <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}" title="\bg_white \hat{x}_{k}" />의 예측값

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;P_{x}^{-}" title="\bg_white P_{x}^{-}" /> : <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;P_{k}" title="\bg_white P_{k}" />의 예측값

- 시스템 모델 : 시스템이 가지는 특성을 반영하는 행렬로 

     A : <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k-1}" title="\bg_white \hat{x}_{k-1}" />(이전 추정값)을 통해 추정값을 예측할 때 사용하는 행렬(상태 행렬)

     H = <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}^{-}" title="\bg_white \hat{x}_{k}^{-}" />을 측정값의 형태로 변환할 때 필요한 행렬(관측 행렬)

     Q = 시스템 노이즈 공분산

     R = 측정값 노이즈 공분산

---

### 1. 예측과정

- 추정값(1)과 오차 공분산 예측(2)

   <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\left\{\begin{matrix}\hat{x}_{k}^{-}&space;=&space;A&space;\hat{x}_{k-1}&space;\triangleright1&space;\\P_{k}^{-}&space;=&space;A&space;P_{k-1}A^{T}&space;&plus;&space;Q&space;\triangleright2\end{matrix}\right." title="\bg_white \left\{\begin{matrix}\hat{x}_{k}^{-} = A \hat{x}_{k-1} \triangleright1 \\P_{k}^{-} = A P_{k-1}A^{T} + Q \triangleright2\end{matrix}\right." />
   
   ```Matlab
     % prediction for estimation value and error of EulerEKF.m
     xp = fx(x,rates,dt);
     Pp = A*P*A' + Q;
     ---------------------------------------------
     % prediction for estimation value and error of RadarEKF.m
     xp = A * x;
     Pp = A*P*A' + Q;
   ```
   
   첫번째 단계인 예측과정에서는 이전단계의 추정값을 사용하여 시스템모델과의 계산으로 추정값을 예측한다.
   이때, 예측한 값의 평균을 기준으로 멀리 떨어져 있는지 알기 위해 이전 오차공분산을 활용하여 새로운 오차공분산을 예측한다.
   
   따라서 오차공분산은 추정값의 정확성을 판단하는 지표가 된다. 이는 정규분포를 따르며 정규분포의 분산이 작으면 추정값이 대부분 평균 근처에 분포한다는 의미이다. 반면에 분산이 큰 경우 추정값의 범위가 넓어져 추정오차가 커지게 된다.

### 2. 추정 과정 : 칼만필터의 최종 결과물인 추정값을 계산하는 과정
#### (PipeLine의 2-4번과정에 해당)
 ```Matlab
     % calculate gain
     K = Pp*H'*inv(H*Pp*H'+R);
     
     % calculate estimation value
     x = xp + K*(z-H*xp);
     
     % calculate error
     P = Pp-(K*H*Pp);
```

- LPF와 유사한 칼만필터

     3번과정에서 행렬 H를 단위행렬로 가정하면 다음과 같이 정리할 수 있다.

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}&space;=&space;(1-K_{k})\hat{x}_{k}^{-}&space;&plus;&space;K_{k}z_{k}" title="\bg_white \hat{x}_{k} = (1-K_{k})\hat{x}_{k}^{-} + K_{k}z_{k}" />

     이는 예측값(<img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;\hat{x}_{k}^{-}" title="\bg_white \hat{x}_{k}^{-}" />)과 측정값(<img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;z_{k}" title="\bg_white z_{k}" />)에 적절한 가중치를 곱한 다음, 두 값을 더해서 최종 추정값을 계산하는 방식을 다르고 있다. 이는 저주파 통과 필터와 매우 비슷한 모습을 띄고 있다.

- 변화하는 가중치, 칼만이득

     <img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;K_{k}&space;=&space;P_{k}^{-}H^{T}(HP_{k}^{-}H^{T}&plus;R)^{-1}" title="\bg_white K_{k} = P_{k}^{-}H^{T}(HP_{k}^{-}H^{T}+R)^{-1}" />
     
     하지만 LPF와 결정적으로 다른점은 가중치(<img src="https://latex.codecogs.com/png.image?\dpi{120}&space;\bg_white&space;K_{k}" title="\bg_white K_{k}" />)가 변화한다는 점이다.
     2번과정에서 보면 시스템 모델을 제외하면 가중치, 즉 칼만이득을 계산하기 위해서 오차공분산이 반영이 되는 것을 알 수 있다. 따라서 오차공분산의 추정오차 크기에 따라 가중치를 가변적  으로 제어하여 최상의 추정값을 추정하게 된다.


     또한 칼만이득의 경우 측정값의 노이즈 R과도 밀접한 연관이 있다. 노이즈가 커지게 되면 칼만이득의 크기가 작아지면서 측정값보다는 예측값을 더 신뢰하게 된다. 반대로 노이즈가 작아 칼만이득이 커지면, 측정값을 신뢰하여 더 큰 가중치를 부여하게 된다.

---

### - Extended Kalman Filter : 비선형 시스템

EKF는 시스템모델이 비선형인 경우 적용가능한 필터이다. 기본적인 알고리즘의 PipeLine은 선형 칼만필터 알고리즘과 동일하다. 다만 비선형인 시스템 모델(상태행렬,관측행렬)을 선형으로 근사시키기 위해 자코비안(Jacobian)을 활용하여 복잡하게 얽혀 있는 식을 미분을 통해 근사 선형식을 만들게 된다.

```Matlab
A = Ajacob(x,rates,dt); %상태행렬이 비선형인경우(EuelrEKF.m)
H = Hjacob2(x); %관측행렬이 비선형인경우(RadarEKF.m)
```

코드에서 볼 수 있듯이 먼저 비선형인 시스템을 선형으로 근사시키는 작업을 한 이후 예측과정과 추정과정을 반복하게 된다.

이 때 자코비안 행렬은 모든 벡터들의 1차 편미분값으로 된 행렬로, 각 행렬의 값은 다변수 함수일 때의 미분값을 의미한다. 자코비안을 구하기 위해 수치해법의 유한차분의 방법 중 정확도가 가장 높은 중심차분을 이용하여 편미분을 하여 구하게 된다.

```Matlab
% To find out first order difference, using central diffrence(numerical method)
function H = partial_diff(f,x,y,z,dx,dy,dz)
% x= phi(roll), y = theta(pitch), z = row(yaw)

dfdx1 = (f(x+dx,y,z) - f(x-dx,y,z))/(2*dx);
dfdx2 = (f(x,y+dy,z) - f(x,y-dy,z))/(2*dy);
dfdx3 = (f(x,y,z+dz) - f(x,y,z-dz))/(2*dz);

H = [dfdx1 dfdx2 dfdx3];
end
```
