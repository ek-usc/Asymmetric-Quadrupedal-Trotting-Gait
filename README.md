# Inter-Diagonal Leg-Pair Phase Asymmetry in the Trotting Gait may Reduce Lateral Drift for a Quadrupedal Robot when Climbing Sloped Inclines with discrete footholds

![](https://github.com/user-attachments/assets/6a3051fb-f376-471a-b2c7-3d6895913be6)

# Research Question
Can applying an offset to the inter-diagonal leg-pair phase, deviating from the canonical π for symmetric gaits, reduce lateral drift for a quadrupedal robot navigating a sloped incline with slanted footholds?

![](https://github.com/user-attachments/assets/005965a4-1350-4c6d-a595-3e9e5e339b45) 

# Methodology
## Inter-diagonal leg-pair phase asymmetry for trot:

𝚫𝛟1 = 𝛟 RR - 𝛟LF ≠ 0 (intra-diagonal leg-pair phase difference, first pair)  

𝚫𝛟2 = 𝛟RF - 𝛟LR ≠ 0 (intra-diagonal leg-pair phase difference, second pair)  

𝚫𝛟𝛂 = 𝛟LR - 𝛟LF ≠ 𝚷 (inter-diagonal leg-pair phase difference)  

𝛟 = 0 when the LF-RR pair enters stance

## Independent Variables

𝛃 (slope incline) ∈ {5°, 10°}  

𝚫𝛟𝛂∈ {140°, 160°, 180°, 200°, 220°} (when 𝛃 = 5°)  

𝚫𝛟𝛂∈ {160°, 170°, 180°, 190°, 200°} (when 𝛃 = 10°)

## Dependent Variables

𝚫s (lateral drift in cm/trial)  

t (time in sec)

(others omitted due to project scope)

## Experimental Setup

(2x5 data points with 3 trials each = 30 total)

Robot starts at fixed point on the board at 𝛟 ≅ 0 (start of LF-RR stance phase), facing uphill

Successful trial ends at 65 cm longitudinal displacement (finish line)

Robot’s x,y position measured from center of Arduino Uno board.

Robot was fine-tuned for each incline, then kept constant for every trial for that incline

Failed trials: x,y recorded when x reaches +/-21cm (robot still on board, but would fall off next gait cycle)



![](https://github.com/user-attachments/assets/3ef8801b-89eb-40ef-8ac9-7518f31cd3be)
![](https://github.com/user-attachments/assets/4b03ae9a-2fda-447e-a9da-c13d331943bb)

 

