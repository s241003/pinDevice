def whichSolenoid(angle,pinN=6):
    elevSoles=[]
    if angle<=22.5 or (angle>=337.5 and angle<=360):
        elevSoles += [1,4] if pinN==6 else [4]
    elif (angle>=22.5 and angle<=67.5):
        elevSoles += [4] if pinN==6 else [6]
    elif (angle>=67.5 and angle<=112.5):
        elevSoles += [5] if pinN==6 else [7]
    elif (angle>=112.5 and angle<=157.5):
        elevSoles += [6] if pinN==6 else [8]
    elif (angle>=157.5 and angle<=202.5):
        elevSoles += [3,6] if pinN==6 else [5]
    elif (angle>=202.5 and angle<=247.5):
        elevSoles += [3] if pinN==6 else [3]
    elif (angle>=247.5 and angle<=292.5):
        elevSoles += [2] if pinN==6 else [2]
    elif (angle>=292.5 and angle<=337.5):
        elevSoles += [1] if pinN==6 else [1]
    return elevSoles

def whichSolenoid_Braille(angle):
    elevSoles=[]

    # elevSolesの値1~8に対応するソレノイドはsole.pngを参照

    if angle<=22.5 or (angle>=337.5 and angle<=360):
        elevSoles += [1,3,5,6] # 前：「ま」
    elif (angle>=22.5 and angle<=67.5):
        elevSoles += [] # 右斜め前：未定
    elif (angle>=67.5 and angle<=112.5):
        elevSoles += [1,2,3,5,6] # 右：「み」
    elif (angle>=112.5 and angle<=157.5):
        elevSoles += [] # 右斜め下：未定
    elif (angle>=157.5 and angle<=202.5):
        elevSoles += [1,4] # 後ろ：「う」
    elif (angle>=202.5 and angle<=247.5):
        elevSoles += [] # 左斜め下：未定
    elif (angle>=247.5 and angle<=292.5):
        elevSoles += [1,2,3,6] # 左：「ひ」
    elif (angle>=292.5 and angle<=337.5):
        elevSoles += [] # 左斜め上：未定

    return elevSoles
