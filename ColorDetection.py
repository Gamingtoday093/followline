from pybricks.ev3devices import ColorSensor

EQUALS_TOLERANCE = 2
LINE_BIAS = 1 / 3
INTERVAL_TOLERANCE_FACTOR = 20 / 40 # 20: observed interval of average value 40

class ColorCompareUsage:
    def __init__(self, useRGB: bool, rgb: tuple[int, int, int], useReflection: bool, reflection: int) -> None:
        self.UseRGB = useRGB
        self.RGB = rgb
        self.UseReflection = useReflection
        self.Reflection = reflection

class ColorHDR:
    def __init__(self, rgb: tuple[int, int, int], reflection: int):
        self.Color = [rgb[0], rgb[1], rgb[2], reflection]

    @classmethod
    def fromColorSensor(cls, sensor: ColorSensor, compare: ColorCompareUsage = None):
        rgb = (0, 0, 0)
        if compare is None or compare.UseRGB:
            rgb = sensor.rgb()
        else:
            rgb = compare.RGB
        reflection = 0
        if compare is None or compare.UseReflection:
            reflection = sensor.reflection()
        else:
            reflection = compare.Reflection

        return cls(rgb, reflection)

    def updateFromColorSensor(self, sensor: ColorSensor, compare: ColorCompareUsage = None):
        rgb = None
        if compare is None or compare.UseRGB:
            rgb = sensor.rgb()
        else:
            rgb = compare.RGB
        
        self.Color[0] = rgb[0]
        self.Color[1] = rgb[1]
        self.Color[2] = rgb[2]

        reflection = 0
        if compare is None or compare.UseReflection:
            reflection = sensor.reflection()
        else:
            reflection = compare.Reflection

        self.Color[3] = reflection

    @staticmethod
    def equals(color1: int, color2: int) -> bool:
        offset = color1 - color2
        interval = (max(color1, color2) * INTERVAL_TOLERANCE_FACTOR) + 1
        return offset <= interval and offset >= -interval
    
    #Detectable colors
    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, ColorHDR):
            return False
        
        if ColorHDR.equals(self.reflection(), __value.reflection()):
            for c in range(len(self.rgb())):
                if not ColorHDR.equals(self.rgb()[c], __value.rgb()[c]):
                    return False
            return True
        return False
    
        return ColorHDR.equals(self.reflection(), __value.reflection())
    
    def almostEqual(self, sensorColor: object, other: object) -> bool:
        return self.__eq__(sensorColor)
        colAverage = float((self.average() + other.average()) / 2) - LINE_BIAS
        #print(str(sensorColor.Color) + " (" + str(sensorColor.average()) + "), " + str(self.Color) + ", " + str(other.Color) + " (" + str(colAverage) + ")")
        if sensorColor.average() <= colAverage:
            return False
        return True

    # Average color
    
    def add(self, value: object):
        for c in range(len(self.Color)):
            self.Color[c] += value.Color[c]
    
    def divide(self, value: int):
        for c in range(len(self.Color)):
            self.Color[c] /= value

    def average(self) -> float:
        tot = 0
        for c in self.rgb():  # self.Color
            tot += c
        tot /= len(self.rgb())  # self.Color
        return tot

    def rgb(self) -> tuple[int, int, int]:
        """ rgb() -> tuple[int, int, int]

            Returns:
            Tuple of reflections for red, green, and blue light, each
            ranging from 0.0% (no reflection) to 100.0% (high reflection).
        """
        return (self.Color[0], self.Color[1], self.Color[2])
    
    def reflection(self) -> int:
        """ reflection() -> int %:
        
            Returns:
            Reflection, ranging from 0% (no reflection) to
            100% (high reflection).
        """
        return self.Color[3]

    def NoneColor(self):
        for c in self.Color:
            if c != 0:
                return False
        return True

    @staticmethod
    def compare(color1, color2) -> ColorCompareUsage:
        """ compare(ColorHDR, ColorHDR) -> ColorCompareUsage: bool, bool, bool

            Returns:
            ColorCompareUsage which consists of 3 bools that describe what sensors are important
        """
        useRGB = False
        useReflection = False

        for c in range(len(color1.rgb())):
            if not ColorHDR.equals(color1.rgb()[c], color2.rgb()[c]):
                useRGB = True
    
        useReflection = not ColorHDR.equals(color1.reflection(), color2.reflection())

        if not useRGB and not useReflection:
            useReflection = True
            print("Failed to find any different comparable Colors!")
        elif useRGB and useReflection:
            useRGB = False
        return ColorCompareUsage(useRGB, color1.rgb(), useReflection, color1.reflection())
        