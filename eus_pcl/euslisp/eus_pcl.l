(load "euspcl.so")

;; Obsolated. all codes below should be rewritten ugin euspcl.
#|
(let* ((lib (load-foreign (format nil "~A/euslib/demo/youhei/libs/pcl_test/libPCLeus.so"
                                  (unix::getenv "CVSDIR")))))
  (defforeign c-calc-spin-image lib "computeSpinImage" (:string :string :integer :integer :float :string) :integer)
  (defforeign c-calc-spin-image-indices lib "computeSpinImageIndices"
    (:string :string :integer :integer :float :string :string :integer) :integer)

  (defforeign c-calc-pfh lib "computePFH" (:string :string :integer :integer :float :string) :integer)
  (defforeign c-calc-pfh-indices lib "computePFHIndices"
    (:string :string :integer :integer :float :string :string :integer) :integer)

  (defforeign c-calc-fpfh lib "computeFPFH" (:string :string :integer :integer :float :string) :integer)
  (defforeign c-calc-fpfh-indices lib "computeFPFHIndices"
    (:string :string :integer :integer :float :string :string :integer) :integer)

  (defforeign c-calc-vfh lib "computeVFH" (:string :string :integer :integer :float :string) :integer)
  (defforeign c-calc-vfh-indices lib "computeVFHIndices"
    (:string :string :integer :integer :float :string :string :integer) :integer)

  (defforeign c-calc-mesh lib "computeMesh" (:string :string :string :integer :float :string) :integer)

  (defforeign c-calc-lms lib "computeLMS" (:string :integer :string :string :string :float) :integer)

  (defforeign c-calc-normal lib "computeNormal" (:string :integer :string :string :float) :integer)

  (defforeign c-extract-planes lib "extractPlanes" (:string :integer :float :integer :integer :string) :integer)

  (defforeign c-extract-planes2 lib "extractPlanes2" (:string :integer :float :integer :integer
                                                              :integer :float :string :string) :integer)

  (defforeign c-concave-plane lib "concavePlane" (:string :integer :string :float :string) :integer)

  (defforeign c-convex-plane lib "convexPlane" (:string :integer :string :string) :integer)

  (defforeign c-extract-euclidean-clusters lib "extractEuclideanClusters"
    (:string :integer :float :integer :integer :string) :integer)
  )


(defun calc-lms (3dp &optional (radius 25.0))
  (let* ((size (send 3dp :size))
         (cvt (instantiate float-vector size))
         (nom (make-matrix size 3))
         (mat (make-matrix size 3))
         (ret (instance 3dpointcloud :init))
         rsize
         )
    (setq rsize (c-calc-lms (array-entity (send 3dp :matrix)) (send 3dp :size)
                            (array-entity mat) (array-entity nom) cvt radius))
    ;; use rsize
    (send ret :matrix mat)
    (send ret :normal nom)
    (send ret :carray (copy-object (send 3dp :carray)))
    ret
    ))

(defun calc-normal (3dp &optional (radius 25.0))
  (let* ((size (send 3dp :size))
         (cvt (instantiate float-vector size))
         (nom (make-matrix size 3))
         )
    (c-calc-normal (array-entity (send 3dp :matrix)) (send 3dp :size)
                   (array-entity nom) cvt radius)
    (send 3dp :normal nom)
    3dp))

(defun calc-feature (3dp &key (feature :spin-image) (indices) (radius 50.0))
  (let* ((ss (send 3dp :size))
         (mat (send 3dp :matrix))
         (nom (send 3dp :normal))
         (isize (if indices (length indices)))
         (hist (case feature
                 (:pfh (make-matrix (if indices isize ss) 125))
                 (:fpfh (make-matrix (if indices isize ss) 33))
                 (:vfh (instantiate float-vector 308))
                 (t (make-matrix (if indices isize ss) 153)))) ;; spin-image
         (func (if indices
                   (case feature
                     (:pfh #'c-calc-pfh-indices)
                     (:fpfh #'c-calc-fpfh-indices)
                     (:vfh #'c-calc-vfh-indices)
                     (t #'c-calc-spin-image-indices))
                 (case feature
                   (:pfh #'c-calc-pfh)
                   (:fpfh #'c-calc-fpfh)
                   (:vfh #'c-calc-vfh)
                   (t #'c-calc-spin-image))))
         )
    (if indices
        (funcall func
                 (array-entity mat)
                 (array-entity nom)
                 ss 1 radius (if (vectorp hist) hist (array-entity hist)) indices isize)
      (funcall func
               (array-entity mat)
               (array-entity nom)
               ss 1 radius (if (vectorp hist) hist (array-entity hist))))
    hist))

(defun make-mesh (3dp &optional (radius 30.0))
  (let* ((mm (send 3dp :matrix))
         (nm (send 3dp :normal))
         (size (send 3dp :size))
         (cvt (instantiate float-vector size))
         (ret (instantiate integer-vector (* size 3 3)))
         rsize
         fs)
    (setq rsize (c-calc-mesh (array-entity mm) (array-entity nm)
                             cvt size radius ret))
    (when (> rsize (* size 3))
      (warn "size exceeded !!~%")
      (setq rsize (* size 3)))

    (dotimes (i rsize)
      (let ((n (c-matrix-row nm (elt ret (* i 3))))
            (a (c-matrix-row mm (elt ret (* i 3))))
            (b (c-matrix-row mm (elt ret (+ (* i 3) 1))))
            (c (c-matrix-row mm (elt ret (+ (* i 3) 2)))))
        (if (> (v. n (v* (v- b a) (v- c a))) 0)
            (push (instance face :init :vertices (list a b c)) fs)
          (push (instance face :init :vertices (list a c b)) fs))
        ;;(push (instance face :init :vertices (list a b c)) fs)
        ))
    ;;ret
    (instance faceset :init :faces fs)
    ))

(defun calc-nearest-neighbor (mat)
  (let* ((size (array-dimension mat 0))
         (dim (array-dimension mat 1))
         kdt
         (p (instantiate float-vector dim))
         (q (instantiate float-vector dim))
         (dlst (instantiate float-vector size))
         (ilst (instantiate integer-vector size))
         ret
         )
    (setq kdt (instance kd-tree :init mat))
    ;;(send kdt :search-corresponding-index

    (dotimes (i size)
      (c-matrix-row mat i p)
      (setq ret (send kdt :nearest-neighbor p :k 2 :return-index t))
      ;;(c-matrix-row ret 1 q)
      (c-matrix-row mat (elt ret 1) q)
      (setf (elt ilst i) (elt ret 1))
      (setf (elt dlst i) (distance p q))
      )
    (list ilst dlst)
    ))

(defun calc-distance-from-other-points (mat &optional (pos 0))
  (let* ((size (array-dimension mat 0))
         (dim (array-dimension mat 1))
         kdt
         (p (instantiate float-vector dim))
         (q (instantiate float-vector dim))
         (dlst (instantiate float-vector size))
         (ilst (instantiate integer-vector size))
         ret lst
         )
    (setq kdt (instance kd-tree :init mat))

    (cond
     ((numberp pos)
      (c-matrix-row mat pos p))
     ((vectorp pos)
      (sys::vector-replace p pos)))
    (setq ret (send kdt :nearest-neighbor p :k size :return-index t))

    (dotimes (i (length ret))
      (c-matrix-row mat (elt ret i) q)
      (push (list (elt ret i) (distance p q)) lst))
    (nreverse lst)
    ))

(defun extract-planes (mat &key (distance-threshold 10.0) (max-plane 10) (min-points 1000))
  (let ((f (instantiate float-vector (* 4 max-plane)))
        (size (array-dimension mat 0))
        ret lst)
    (setq ret (c-extract-planes (array-entity mat) size
                                distance-threshold min-points max-plane f))
    (dotimes (i ret)
      (push (subseq f (* 4 i) (* 4 (1+ i))) lst))
    (nreverse lst)))

(defun extract-planes2 (pc &key (extract-not-plane-points) (distance-threshold 10.0)
                           (max-plane 10) (min-points 1000) (minsize 500) (tolerance 0.02))
  (let* ((mat (send pc :points))
         (size (array-dimension mat 0))
         (f (instantiate float-vector (* 4 max-plane)))
         (indices (instantiate integer-vector size))
         ret plst)
    (setq ret (c-extract-planes2 (array-entity mat) size
                                 distance-threshold min-points max-plane
                                 minsize tolerance
                                 f indices))
    (dotimes (i ret)
      (push (subseq f (* 4 i) (* 4 (1+ i))) plst))
    (nreverse plst) ;; coeff

    (let (idx-lst)
      (dotimes (i ret)
        (let (idx)
          (dotimes (j size)
            (if (= (elt indices j) i)
                (push j idx)))
          (setq idx (reverse idx))
          (push idx idx-lst)))
      (setq idx-lst (nreverse idx-lst))

      (let ((pts
             (mapcar #'(lambda (idxs pl)
                         (let ((p (send pc :filter-with-indices idxs :create t)))
                           (setf (get p :plane-coeff) pl)
                           p))
                     idx-lst plst)))
        (when extract-not-plane-points
          (let  (nidxs npt)
            (dotimes (j size)
              (if (= (elt indices j) -1)
                  (push j nidxs)))
            (setq nidxs (reverse nidxs))
            (setq npt (send pc :filter-with-indices nidxs :create t))
            (push npt pts)
            ))
        pts))
    ))

(defun extract-clusters (pc &key (maxsize 200000) (minsize 500) (tolerance 0.02))

  (let* ((mat (send pc :points))
         (size (array-dimension mat 0))
         (indices (instantiate integer-vector size))
         cluster-size plst)

    (setq cluster-size (c-extract-euclidean-clusters (array-entity mat) size
                                                     tolerance minsize maxsize
                                                     indices))

    (let (idx-lst)
      (dotimes (cls cluster-size)
        (let (idx)
          (dotimes (j size)
            (if (= (elt indices j) cls)
                (push j idx)))
          (setq idx (reverse idx))
          (push idx idx-lst)))
      (setq idx-lst (nreverse idx-lst))

      (mapcar #'(lambda (idxs)
                  (send pc :filter-with-indices idxs :create t))
              idx-lst)
      )))

(defun extract-plane-points (3dp coeff-lst &key (create t) (distance 10.0) (negative))
  (let ((ret 3dp)
        pl-lst)
    (when create
      (setq ret (instance 3dpointcloud :init))
      (send ret :copy-from 3dp))
    (unless (listp coeff-lst)
      (setq coeff-lst (list coeff-lst)))
    (dolist (coeff coeff-lst)
      (push (instance plane :init (subseq coeff 0 3) (elt coeff 3)) pl-lst))
    (send ret :nfilter
          :key #'(lambda (p)
                   (let ((r nil))
                     (dolist (pl pl-lst)
                       (when (< (abs (send pl :distance p)) distance)
                         (setq r t)
                         (return)))
                     r))
          :negative negative)
    ret))
(defun extract-plane-points (3dp coeff-lst &key (create t) (distance 10.0) (negative))
  (let ((ret 3dp)
        pl-lst)
    (when create
      (setq ret (instance pointcloud :init))
      (send ret :copy-from 3dp))
    (unless (listp coeff-lst)
      (setq coeff-lst (list coeff-lst)))
    (dolist (coeff coeff-lst)
      (push (instance plane :init (subseq coeff 0 3) (elt coeff 3)) pl-lst))
    (send ret :nfilter
          :key #'(lambda (p)
                   (let ((r nil))
                     (dolist (pl pl-lst)
                       (when (< (abs (send pl :distance p)) distance)
                         (setq r t)
                         (return)))
                     r))
          :negative negative)
    ret))

(defun extract-each-plane-points (3dp coeff-lst
                                      &key (create t) (distance 10.0) (negative))
  (let (pl-lst)
    (unless (listp coeff-lst)
      (setq coeff-lst (list coeff-lst)))
    (dolist (coeff coeff-lst)
      (push (instance plane :init (subseq coeff 0 3) (elt coeff 3)) pl-lst))
    (setq pl-lst (nreverse pl-lst))
    (mapcar
     #'(lambda (pl)
         (let ((ret
                (send 3dp :nfilter
                      :key #'(lambda (p)
                               (< (abs (send pl :distance p)) distance))
                      :create t)))
           (setf (get ret :plane) pl)
           ret))
     pl-lst)
    ))

(defun concave-plane (mat coeff &key (alpha 0.1))
  (let* ((size (array-dimension mat 0))
         (ret (make-matrix size 3))
         rsize  pts)
    (setq rsize (c-concave-plane (array-entity mat) size
                                 coeff alpha (array-entity ret)))
    (setq pts (make-matrix rsize 3))
    (sys::vector-replace (array-entity pts) (array-entity ret))
    pts))
(defun convex-plane (mat coeff)
  (let* ((size (array-dimension mat 0))
         (ret (make-matrix size 3))
         rsize pts)
    (setq rsize (c-convex-plane (array-entity mat) size
                                coeff (array-entity ret)))
    (setq pts (make-matrix rsize 3))
    (sys::vector-replace (array-entity pts) (array-entity ret))
    pts))

;;
;;
;;
(defun make-on-plane-points (3dp)
  (setq vcds (send (get 3dp :view-coords) :copy-worldcoords))
  (setq v3dp (send 3dp :transform-points (send vcds :inverse-transformation) :create t))

  (setq coeff-lst (extract-planes (send v3dp :matrix)
                                  :distance-threshold 10.0
                                  :max-plane 5
                                  :min-points 400))

  ;; table points ;; ad-hook
  (setq coeff
        (cond
         ((> 850 (elt (car coeff-lst) 3) 550)
          (car coeff-lst))
         ((> 850 (elt (cadr coeff-lst) 3) 550)
          (cadr coeff-lst))))

  ;;(setq coeff (cadr coeff-lst))
  (setq pl (instance plane :init (subseq coeff 0 3) (elt coeff 3)))

  (setq p-pl (extract-plane-points v3dp (list coeff)))
  (setq cent (send p-pl :centroid))
  (setq pcds (make-aligned-coords (send pl :normal) :pos cent :axis :z))

  (setq 3dp-onp (send v3dp :nfilter :create t
                      :key #'(lambda (x) (> (send pl :distance x) 15.0))))
  (send 3dp-onp :transform-points (send pcds :inverse-transformation))

  (setq gv (gl::make-voxel-grid 3dp-onp :grid-size 25 :fix-size t :xy-plane t))
  ;;(send gv :labeling 0.5)
  (setq lab-lst (send gv :voxel-labeling 0.5))
  (setq lab-str (send gv :voxel-labeling-mode-array))

  (setq 3dp-lst nil)
  (dolist (lab lab-lst)
    (push (send gv :extract-point-indices 3dp-onp lab-str lab :create t) 3dp-lst))
  (nreverse 3dp-lst)
  )

(defun make-color-ring (num &key (ring 8) )
  (let ((intensity 1.0)
        (saturation 1.0)
        ret)
    (dotimes (j (1+ (/ num ring)))
      (dotimes (i ring)
        (push
         (scale (/ 1.0 255.0)
                (coerce
                 (his2rgb (* i (/ 360.0 ring)) intensity saturation)
                 float-vector))
         ret))
      (if (= (mod j 2) 0)
          (decf intensity 0.4)
        (decf saturation 0.4)))
    (subseq (nreverse ret) 0 num)
    ))

(defmethod pointcloud
  (:find-nearest-raw (3dp)
   (let ((mat (if (derivedp 3dp float-vector)
                  (let ((m (make-matrix 1 3)))
                    (fvector-replace (array-entity m) 3dp)
                    m)
                (send 3dp :points)))
         )
     (user::c-kdtree-k-search-matrix-raw
      parray mat 1 single-float-epsilon))
   ))
|#
