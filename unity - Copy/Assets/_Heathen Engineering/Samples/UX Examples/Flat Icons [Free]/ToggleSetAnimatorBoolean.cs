using UnityEngine;

namespace HeathenEngineering.UX.Samples
{
    public class ToggleSetAnimatorBoolean : MonoBehaviour
    {
        public Animator animator;
        public string booleanName;

        public void SetBoolean(bool value)
        {
            animator.SetBool(booleanName, value);
        }
    }
}
