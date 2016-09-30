(load "models/foldable-desk-object.l")
(load "models/kettle-object.l")
(load "models/uchida-shelf-1100-object.l")
(load "models/coe-800-shelf-object.l")
(load "models/gifuplastic-900-cart-object.l")
(load "models/room73b2-plus-590-locker-object.l")
(load "models/room73b2-karimoku-table-object.l")

(setq *kettle* (kettle))
(setq *shelf1* (uchida-shelf-1100))
(setq *shelf2* (coe-800-shelf))
(setq *cart* (gifuplastic-900-cart))
(setq *locker* (room73b2-plus-590-locker))
(setq *table* (room73b2-karimoku-table))
(setq *desk* (foldable-desk))

(setq *floor* (make-cube 10000 10000 10))
(setf (get *floor* :face-color) (float-vector 0.8 0.8 0.8))
(send *floor* :translate (float-vector 0 0 -5))

(send *shelf1* :translate (float-vector 0 -2000 0))
(send *shelf1* :rotate pi/2 :z)

(send *locker* :translate (float-vector -1500 0 0))

(send *desk* :translate (float-vector 0 1500 0))

;;
(send *shelf2* :locate (float-vector 2000 -1500 0) :world)
(send *cart* :locate (float-vector 2000 1500 0) :world)
;;
(send *kettle* :locate (float-vector 0 0 700) :world)

(setq *objs* (list
*kettle*
*shelf1*
*shelf2*
*cart*
*locker*
*table*
*desk*
*floor*
))

(objects *objs*)

#| ;; large camera
(setq *camera-model*
      (make-camera-from-param :pwidth 640 :pheight 360
                              :fx 400 :fy 400
                              :cx 319.5 :cy 179.5 :name "camtest"
                              :create-viewer t))
|#
(setq *camera-model*
      (make-camera-from-param :pwidth 160 :pheight 120
                              :fx 100 :fy 100
                              :cx 79.5 :cy 59.5 :name "camtest"
                              :create-viewer t))

(send *camera-model* :rotate -2.2 :x)
(send *camera-model* :rotate (- pi 0.8) :z :world)
(send *camera-model* :locate (float-vector 800 600 1300) :world)

(defun get-points ()
  (send *camera-model* :draw-objects *objs*)

  (let ((ret (send *camera-model* :get-image :with-points t :with-colors t)))
    (setq *image* (car ret))
    (setq *points* (cdr ret))
    )
  ;; transform origin of point cloud
  (send *points* :transform (send *camera-model* :worldcoords))
  *points*)

#|
;;; demo octomap
(get-points)
;; clear near points
(send *points* :filter :key #'(lambda (p) (> (norm p) 100)))
(require :eus_pcl "package://eus_pcl/euslisp/eus_pcl.so")

(setq ptr (pcl::octomap-create 30))

(pcl::octomap-add-points ptr *points* #f(0 0 0))

(setq a-pc (car (pcl::octomap-read-nodes ptr)))
(setq u-pc (pcl::octomap-read-unknown ptr))

(setq points-lst nil)
(while (/= (send a-pc :size) 0)
(let ((idx (random (send a-pc :size)))
      pt)
  (setq pt (matrix-row (send a-pc :points) idx))
  (push pt points-lst)
  (send a-pc :filter :key #'(lambda (p) (> (distance p pt) 60)))
  (pprint (send a-pc :size))
  ))

(progn
  (setq qq (instance pointcloud :init))
  (send qq :points points-lst)
  (send qq :point-color #f(1 0 0)))
|#
