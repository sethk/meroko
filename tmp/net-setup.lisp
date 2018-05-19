;;; -*- Mode:Common-Lisp; Package:IP; Base:10 -*-

(defun add-default-router (host)
  (let* ((host (parse-ip-host-spec host))
	 (addr (first (send host :network-address-list :ip))))
    (push-end (allocate-ip-route addr (first net:controller-list) addr nil 0)
	      ip-routing-table)))

(defun set-arp-to-mine (host)
  (let* ((host (parse-ip-host-spec host))
	 (addr (first (send host :network-address-list :ip)))
	 (mac (send (first net:controller-list) :ethernet-address)))
    (unless (assoc addr ethernet:ip-ether-address-translations :test #'eql)
      (push (ethernet:make-ip-ether-cache-element :ip-addr addr :ether-addr mac)
	    ethernet:ip-ether-address-translations))))

(progn
  (set-arp-to-mine "192.168.42.1")
  (add-default-router "192.168.42.1"))
