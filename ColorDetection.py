from pybricks.ev3devices import ColorSensor

EQUALS_TOLERENCE = 10

class ColorCompareUsage:
    def __init__(self, useRGB, useReflection, useAmbient) -> None:
        self.UseRGB = useRGB
        self.UseReflection = useReflection
        self.UseAmbient = useAmbient

class ColorHDR:
    def __init__(self, rgb: tuple[int, int, int], reflection: int, ambient: int):
        self.Color = (rgb[0], rgb[1], rgb[2], reflection, ambient)

    @classmethod
    def fromColorSensor(cls, sensor: ColorSensor):
        return cls(sensor.rgb(), sensor.reflection(), sensor.ambient())

    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, ColorHDR):
            return False
        
        for c in range(len(self.Color)):
            offset = self.Color[c] - __value.Color[c]
            if offset > EQUALS_TOLERENCE or offset < -EQUALS_TOLERENCE:
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
    def compare(color1, color2) -> (bool, bool, bool):
        return True, False, False

a = ColorHDR((0, 0, 0), 0, 0)
b = ColorHDR((0, 0, 0), 0, 0)

print(ColorHDR.compare(a, b))