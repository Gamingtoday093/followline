from pybricks.ev3devices import ColorSensor

EQUALS_TOLERENCE = 10

class ColorCompareUsage:
    def __init__(self, useRGB: bool, useReflection: bool, useAmbient: bool) -> None:
        self.UseRGB = useRGB
        self.UseReflection = useReflection
        self.UseAmbient = useAmbient

class ColorHDR:
    def __init__(self, rgb: tuple[int, int, int], reflection: int, ambient: int):
        self.Color = (rgb[0], rgb[1], rgb[2], reflection, ambient)

    @classmethod
    def fromColorSensor(cls, sensor: ColorSensor):
        return cls(sensor.rgb(), sensor.reflection(), sensor.ambient())

    @staticmethod
    def equals(color1: int, color2: int) -> bool:
        offset = color1 - color2
        return offset <= EQUALS_TOLERENCE and offset >= -EQUALS_TOLERENCE
    
    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, ColorHDR):
            return False
        
        for c in range(len(self.Color)):
            offset = self.Color[c] - __value.Color[c]
            if offset > EQUALS_TOLERENCE or offset < -EQUALS_TOLERENCE:
                return False
            if not ColorHDR.equals(self.Color[c], __value.Color[c]):
                return False

        return True
    
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
    
    def ambient(self) -> int:
        """ ambient() -> int: %

            Returns:
            Ambient light intensity, ranging from 0% (dark)
            to 100% (bright).
        """
        return self.Color[4]
    
    @staticmethod
    def compare(color1, color2) -> ColorCompareUsage:
        useRGB = False
        useReflection = False
        useAmbient = False

        for c in range(len(color1.rgb())):
            if not ColorHDR.equals(color1.rgb()[c], color2.rgb()[c]):
                useRGB = True

        useReflection = not ColorHDR.equals(color1.reflection(), color2.reflection())

        useAmbient = not ColorHDR.equals(color1.ambient(), color2.ambient())

        return ColorCompareUsage(useRGB, useReflection, useAmbient)
        