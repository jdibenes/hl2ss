namespace Dummiesman
{
    public static class StringExtensions
    {
        public static string Clean(this string str)
        {
            string rstr = str.Replace('\t', ' ');
            while (rstr.Contains("  "))
                rstr = rstr.Replace("  ", " ");
            return rstr.Trim();
        }
    }
}
