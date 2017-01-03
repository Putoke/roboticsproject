using Emgu.CV;

namespace DobotClientDemo
{
    class Card
    {
        private Mat image;
        private int color;
        private int value;

        public Card(Mat image, int color, int value)
        {
            this.image = image;
            this.color = color;
            this.value = value;
        }

        public Mat GetImage()
        {
            return image;
        }

        public int GetColor()
        {
            return color;
        }

        public int GetValue()
        {
            return value;
        }
    }
}