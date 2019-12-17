# 相机属性

1. Node Name:OffsetX Type:Integer 相机x方向偏移量
2. Node Name:OffsetY Type:Integer 相机y方向偏移量
3. Node Name:ReverseX Type:Boolean 水平反转 True or False
4. Node Name:ReverseY Type:Boolean 垂直反转 True or False
5. Node Name:PixelFormat Type:Enumeration 像素格式
        
        格式参数:

        Enum Entry Name:Mono8
        Enum Entry Value:0x01080001
        Enum Entry Name:Mono10
        Enum Entry Value:0x01100003
        Enum Entry Name:Mono12
        Enum Entry Value:0x01100005
        Enum Entry Name:RGB8Packed
        Enum Entry Value:0x02180014
        Name:YUV422_YUYV_Packed
        Enum Entry Value:0x02100032
        Name:YUV422Packed
        Enum Entry Value:0x0210001F
        Enum Entry Name:BayerRG8
        Enum Entry Value:0x01080009
        Enum Entry Name:BayerRG10
        Enum Entry Value:0x0110000d
        Name:BayerRG10Packed
        Enum Entry Value:0x010C0027
        Enum Entry Name:BayerRG12
        Enum Entry Value:0x01100011
        Name:BayerRG12Packed
        Enum Entry Value:0x010C002B
6.  Name:AcquisitionFrameRate
Type:Float 帧率

        Min:0.1
        Max:100000
7. Node Name:TriggerMode
Type:Enumeration 触发模式
        Enum Entry Name:Off
        Enum Entry Value:0
        
        Enum Entry Name:On
        Enum Entry Value:1
8. Node Name:ExposureTime
Type:Float曝光时间
        
        Min:40
        Max:9.99984e+06
9. Node Name:Gain
Type:Float增益

        Min:0
        Max:15.0062
10. Node Name:GainAuto
Type:Enumeration自动增益

        Enum Entry Name:Off
        Enum Entry Value:0
        Enum Entry Name:Once
        Enum Entry Value:1
11. Node Name:BalanceWhiteAuto
Type:Enumeration白平衡

        Enum Entry Name:Off
        Enum Entry Value:0
12. Node Name:Gamma
Type:Float伽马矫正

        Min:0
        Max:4
13. Node Name:GammaSelector
Type:Enumeration

        Enum Entry Name:User
        Enum Entry Value:1
        Enum Entry Name:sRGB
        Enum Entry Value:2

14. Node Name:GammaEnable
Type:Boolean 伽马矫正使能
        
        true or false
        要想调整伽马，必须先使能
15. Node Name:Sharpness
Type:Integer 清晰度
        
        Min:0
        Max:100

16. Node Name:SharpnessEnable
Type:Boolean清晰度使能
        
        true or false 
17. Node Name:HueEnable
Type:Boolean色相使能
18. Node Name:Hue
Type:Integer
        Min:0
        Max:255
19. Node Name:SaturationEnable
Type:Boolean 饱和度使能
20. Node Name:Saturation
Type:Integer

        Min:0
        Max:255
21.
