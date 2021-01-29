(load "package://eus_pcl/euslisp/eus-pcl.l")
;;(load "space-sampler.l") ;; jskgeo.l
;; created-by-spacesampler (geo::s2spacesample level=1)
(setq *normalized-sample*
      (list
       #f(0.666667 -0.666667 -0.333333)
       #f(0.285235 -0.688619 -0.666667)
       #f(0.688619 -0.285235 -0.666667)
       #f(0.282597 -0.282597 -0.916667)
       #f(-0.666667 -0.666667 -0.333333)
       #f(-0.688619 -0.285235 -0.666667)
       #f(-0.285235 -0.688619 -0.666667)
       #f(-0.282597 -0.282597 -0.916667)
       #f(-0.666667 0.666667 -0.333333)
       #f(-0.285235 0.688619 -0.666667)
       #f(-0.688619 0.285235 -0.666667)
       #f(-0.282597 0.282597 -0.916667)
       #f(0.666667 0.666667 -0.333333)
       #f(0.688619 0.285235 -0.666667)
       #f(0.285235 0.688619 -0.666667)
       #f(0.282597 0.282597 -0.916667)
       #f(-1.847957e-15 -0.942809 0.333333)
       #f(-0.382683 -0.92388 6.123234e-17)
       #f(0.382683 -0.92388 6.123234e-17)
       #f(-1.847957e-15 -0.942809 -0.333333)
       #f(-0.942809 1.154608e-16 0.333333)
       #f(-0.92388 0.382683 6.123234e-17)
       #f(-0.92388 -0.382683 6.123234e-17)
       #f(-0.942809 1.154608e-16 -0.333333)
       #f(5.773040e-17 0.942809 0.333333)
       #f(0.382683 0.92388 6.123234e-17)
       #f(-0.382683 0.92388 6.123234e-17)
       #f(5.773040e-17 0.942809 -0.333333)
       #f(0.942809 0.0 0.333333)
       #f(0.92388 -0.382683 6.123234e-17)
       #f(0.92388 0.382683 6.123234e-17)
       #f(0.942809 0.0 -0.333333)
       #f(-7.833407e-16 -0.399653 0.916667)
       #f(0.285235 -0.688619 0.666667)
       #f(0.688619 -0.285235 0.666667)
       #f(0.666667 -0.666667 0.333333)
       #f(-0.399653 4.894333e-17 0.916667)
       #f(-0.688619 -0.285235 0.666667)
       #f(-0.285235 -0.688619 0.666667)
       #f(-0.666667 -0.666667 0.333333)
       #f(2.447167e-17 0.399653 0.916667)
       #f(-0.285235 0.688619 0.666667)
       #f(-0.688619 0.285235 0.666667)
       #f(-0.666667 0.666667 0.333333)
       #f(0.399653 0.0 0.916667)
       #f(0.688619 0.285235 0.666667)
       #f(0.285235 0.688619 0.666667)
       #f(0.666667 0.666667 0.333333)))

(defun octomap-demo (&optional (name "sushi-cup") (resolution 2))
  (load (format nil "models/~A-object.l" name))
  (setq *obj* (eval (read-from-string (format nil "(~A)" name))))
  (objects (list *obj*))

  (setq *camera-model*
        (make-camera-from-param :pwidth 160 :pheight 160
                                :fx 100 :fy 100
                                :cx 79.5 :cy 59.5 :name "camtest"
                                :create-viewer t))

  (create-pointcloud-octomap *obj* :camera-model *camera-model* :resolution resolution)
  )

(defun create-objects-points-with-camera (target-obj &key (camera-model) (offset-scale 3) (add-normal))
  (unless camera-model
    (setq camera-model
          (make-camera-from-param :pwidth 160 :pheight 160
                                  :fx 100 :fy 100
                                  :cx 79.5 :cy 59.5 :name "camtest"
                                  :create-viewer t)))
  ;;
  (let* ((bb (make-bounding-box (flatten (send-all (send target-obj :bodies) :vertices)) 0.0))
         (cent (send bb :center))
         (radius (* 0.5 (norm (send bb :diagonal))))
         results
         )
    ;;
    (let ((sample-pos ;;(geo::s2spacesample :scale (* radius offset-scale))
           (mapcar #'(lambda (p) (scale (* radius offset-scale) p)) *normalized-sample*)
           ))
      ;; move sample-pos relative to center
      (dolist (p sample-pos) (v+ p cent p))

      ;;
      (dolist (p sample-pos)
        (let* ((zaxis (v- cent p))
               (cds (orient-coords-to-axis (make-coords) zaxis :z 1e-9)))
          (send cds :locate p :world)
          ;;(send cds :draw-on :flush t)
          (send camera-model :newcoords cds)
          (send camera-model :worldcoords)
          (send camera-model :draw-objects target-obj)
          (unix::usleep (* 40 1000));; 40ms sleep wait for refresh
          ;;(do-until-key (x::window-main-one))
          (let ((ret (send camera-model :get-image :with-points t :with-colors t))
                img pts
                )
            (setq img (car ret)
                  pts (cdr ret))
             ;; remove nearest point
            (send pts :filter :key #'(lambda (p) (> (norm p) 1.0)) :create nil)
            (if add-normal
                (eus_pcl::calc-normals pts :ksearch 8))
            ;;
            (send pts :transform (send camera-model :worldcoords))
            (push pts results)
            )
          ))
      )
    results))

(defun create-pointcloud-octomap (target-obj &key (resolution 2.0) (camera-model) (interactive t))
  (setq *result-points* (create-objects-points-with-camera target-obj :camera-model camera-model))
  (send-all *result-points* :point-size 6)
  (send *irtviewer* :objects *result-points*)
  (send *irtviewer* :draw-objects)
  (when interactive
    (warn ";; input key...~%")
    (do-until-key (x::window-main-one)))

  (setq *omap* (instance eus_pcl::octomap :init :resolution resolution :type :color-octree))

  (warn ";; wait for creating octomap .")
  (dolist (r *result-points*)
    (let ((p (copy-seq (send r :worldpos)))
          pts)
      (setq pts (send r :move-origin-to (make-coords) :create t))
      (send pts :put :view-origin p)
      ;;(pprint :add)
      (send *omap* :add-points pts :origin p)
      (warn ".")
      ))
  (warn "~%")

  (send *omap* :update-visual-color :transparent nil)
  (send *irtviewer* :objects (list *omap*))
  (send *irtviewer* :draw-objects)
  (when interactive
    (warn ";; input key...~%")
    (do-until-key (x::window-main-one)))
  (unless interactive
    (return-from create-pointcloud-octomap))

  (send *omap* :update-visual-color :transparent nil :depth -1)
  (send *irtviewer* :draw-objects)
  (warn ";; input key...~%")
  (do-until-key (x::window-main-one))

  (send *omap* :update-visual-color :transparent nil :depth -2)
  (send *irtviewer* :draw-objects)
  (warn ";; input key...~%")
  (do-until-key (x::window-main-one))

  (send *omap* :update-visual-color :transparent nil :depth -3)
  (send *irtviewer* :draw-objects)
  (warn ";; input key...~%")
  (do-until-key (x::window-main-one))

  (send *omap* :update-visual-color :transparent nil :depth -4)
  (send *irtviewer* :draw-objects)
  )

(warn "
(octomap-demo \"suchi-cup\" 2.0)
(octomap-demo \"arrow\" 2.0)
(octomap-demo \"kettle\" 4.0)
(octomap-demo \"white-bread-salad-dish\" 4.0)
(octomap-demo \"chair\" 10.0)
(octomap-demo \"foldable-desk\" 10.0)
(octomap-demo \"gifuplastic-900-cart\" 14.0)
")

#| ;; check-points
(dolist (r *result-points*)
  (send *irtviewer* :objects (list r))
  (send *irtviewer* :draw-objects)
  (do-until-key (x::window-main-one)))
|#

#|
;; marching-cube demo
(setq g0 (make-icosahedron 100.0))
(setq g1 (make-gdome g0))
(setq bd (instance bodyset :init (make-cascoords) :bodies (list g1)))

(setq *camera-model*
        (make-camera-from-param :pwidth 160 :pheight 160
                                :fx 100 :fy 100
                                :cx 79.5 :cy 59.5 :name "camtest"
                                :create-viewer t))
(setq *result-points*
      (create-objects-points-with-camera bd :camera-model *camera-model* :add-normal t))

(setq in-points
      (let ((r (mapcar #'(lambda (pc) (send pc :convert-to-world :create t)) *result-points*)))
        (send (car r) :append (cdr r))))


(setq fs (euc_pcl::marching-cube-surface-reconstruction in-points))

(objects (list fs))
|#
