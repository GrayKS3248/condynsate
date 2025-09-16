import time
import random
from condynsate.animator import animator_cv2 as a

if __name__ == "__main__":
    animator = a.Animator(1.0)
    animator.add_subplot(n_artists=1,
                         subplot_type='line',
                         title='Test',
                         x_label='test [test]',
                         y_label='test [test]',
                         colors=['m',],
                         line_widths=[2.5,],
                         line_styles=['-',])
    animator.start_animator()
    for i in range(1000):
        time.sleep(0.01)
        animator.add_datum(0, 0, i, random.randint(-7,7))
    animator.terminate()
    print('done')
    