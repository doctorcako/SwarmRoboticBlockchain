(function () {
	"use strict";

	/**
	 * From underscore.js (https://underscorejs.org/#debounce)
	 *
	 * Returns a function, that, as long as it continues to be invoked, will not
	 * be triggered. The function will be called after it stops being called for
	 * N milliseconds. If `immediate` is passed, trigger the function on the
	 * leading edge, instead of the trailing.
	 *
	 * @param {Function} func      function to execute
	 * @param {int}      wait      delay until execution
	 * @param {bool}     immediate pass true to execute without delay
	 * @return {Function} debounced function
	 */
	function debounce(func, wait, immediate) {
		var timeout;
		return function () {
			var context = this,
				args = arguments;
			var later = function () {
				timeout = null;
				if (!immediate) {
					func.apply(context, args);
				}
			};
			var callNow = immediate && !timeout;
			clearTimeout(timeout);
			timeout = setTimeout(later, wait);
			if (callNow) {
				func.apply(context, args);
			}
		};
	}

	// NodeList.forEach
	if (window.NodeList && !NodeList.prototype.forEach) {
		NodeList.prototype.forEach = Array.prototype.forEach;
	}
	// HTMLCollection.forEach
	if (window.HTMLCollection && !HTMLCollection.prototype.forEach) {
		HTMLCollection.prototype.forEach = Array.prototype.forEach;
	}

	var isMobileSafari = /iP(ad|hone|od).+Version\/[\d\.]+.*Safari/i.test(navigator.userAgent);
	var isIE = !!window.document.documentMode;

	// Add browser classes
	if (isMobileSafari) document.documentElement.classList.add('safari-mobile');
	if (isIE) document.documentElement.classList.add('ie');

	/**
	 * Desktop Menu
	 */
	if (!isIE) {
		var header_submenus = document.querySelectorAll('header .nav-menu .menu-item-has-children');

		// Submenu mouse over/leave
		header_submenus.forEach(function (submenu) {
			submenu.addEventListener('mouseover', function (event) {
				var self = this;

				// Hide previous showed submenus
				document.querySelectorAll('header .nav-menu .selected-menu').forEach(function (prev) {
					if (self !== prev && self.parentNode.parentNode.parentNode !== prev) {
						prev.classList.remove('selected-menu');
					}
				})

				this.classList.add('selected-menu');
			}, true);

			// Hide on leave submenu
			submenu.addEventListener('mouseleave', function (event) {
				this.classList.remove('selected-menu');
			}, true);
		});

		var header_menus = document.querySelectorAll('header .nav-menu');

		// Menu on <a> focus trigger submenu mouseover/mouseleave (for keyboard tab navigation)
		header_menus.forEach(function (menu) {
			menu.addEventListener('focus', function (event) {
				var a = event.target;
				if (a.nodeName === 'A') {
					if (a.parentNode.classList.contains('menu-item-has-children')) {
						a.parentNode.dispatchEvent(new Event('mouseover'));
					} else if (a.parentNode.parentNode.parentNode.parentNode.classList.contains('menu-item-has-children')) {
						a.parentNode.parentNode.parentNode.parentNode.dispatchEvent(new Event('mouseover'));
					} else if (menu.querySelector('.selected-menu')) {
						menu.querySelector('.selected-menu').dispatchEvent(new Event('mouseleave'));
					}
				}
			}, true);
		});
	}

	/**
	 * Mobile Menu
	 */

	// Fill Mobile Menu
	var mobile_nav = document.querySelector('.mobile-menu .mm-content');
	mobile_nav.appendChild(document.querySelector('.top-tabs').cloneNode(true));
	mobile_nav.appendChild(document.querySelector('.search-form').cloneNode(true));
	['secondary-menu', 'primary-menu', 'top-menu'].forEach(function (id) {
		var clone = document.getElementById(id).cloneNode(true);
		clone.removeAttribute('id');
		mobile_nav.appendChild(clone);
	});
	mobile_nav.appendChild(document.querySelector('.top-languages .languages-select').cloneNode(true));

	var menu_toggle = document.querySelector('.menu-toggle');
	var menu_search = document.querySelector('.menu-search');
	var menu_close = document.querySelector('.mobile-menu .mm-close');
	var menu_prev = document.querySelector('.mobile-menu .mm-prev');
	var submenu_toggles = mobile_nav.querySelectorAll('.menu-item-has-children');

	menu_toggle.addEventListener('click', toggleMobileMenu);
	menu_close.addEventListener('click', toggleMobileMenu);
	menu_search.addEventListener('click', function () {
		toggleMobileMenu();
		mobile_nav.querySelector('.search-field').focus();
	});

	function toggleMobileMenu() {
		var selecteds = mobile_nav.querySelectorAll('.selected-menu');

		document.body.classList.toggle('show-mobile-nav');
		menu_prev.classList.remove('show');
		selecteds.forEach(function (selected) {
			selected.classList.remove('selected-menu');
		});

		if (menu_toggle.getAttribute('aria-expanded') === 'true') {
			menu_toggle.setAttribute('aria-expanded', 'false');
		} else {
			menu_toggle.setAttribute('aria-expanded', 'true');
		}
	}

	function toggleMobileSubmenu(event) {
		if (!this.classList.contains('selected-menu')) {
			event.preventDefault();
			this.classList.add('selected-menu');
			menu_prev.classList.add('show');
		}
	}

	submenu_toggles.forEach(function (submenu_toggle) {
		submenu_toggle.addEventListener('click', toggleMobileSubmenu, true);
	});

	menu_prev.addEventListener('click', function () {
		var selecteds = mobile_nav.querySelectorAll('.selected-menu');
		selecteds[selecteds.length - 1].classList.remove('selected-menu');

		if (selecteds.length === 1) {
			menu_prev.classList.remove('show');
		}
	});

	/**
	 * Header appereance on scroll
	 */
	var last_scroll = 0;
	var capture_scroll_up = true;

	window.addEventListener('scroll', debounce(function () {
		var scroll = document.body.scrollTop || document.documentElement.scrollTop;
		var progres_header = document.getElementById('progress-header');
		document.body.classList.toggle('scrolled', scroll > 144);
		document.body.classList.toggle('scroll-up', capture_scroll_up && scroll < last_scroll);
		progres_header && progres_header.classList.toggle('progress-header--show', scroll >= 300);
		last_scroll = scroll;
	}), 10);

	/**
	 * Scroll progress
	 */
	var read_progress = document.getElementById('read-progress');

	if (read_progress) {
		var header_share = document.querySelector('.progress-header-share');

		window.addEventListener('scroll', debounce(function () {
			read_progress.max = document.documentElement.scrollHeight - document.documentElement.clientHeight;
			read_progress.value = document.body.scrollTop || document.documentElement.scrollTop;

			// Hide share on mobile scroll
			if (navigator.maxTouchPoints > 0) {
				header_share.classList.remove('progress-header-share--show');
			}
		}, 10));

		if (navigator.maxTouchPoints > 0) {
			// Mobile toggle on tap
			header_share.addEventListener('touchstart', function () {
				header_share.classList.toggle('progress-header-share--show');
			});
		} else {
			// Desktop toggle on mouseover/mouseleave
			header_share.addEventListener('mouseover', function () {
				header_share.classList.add('progress-header-share--show');
			});
			header_share.addEventListener('mouseleave', function (event) {
				if (event.target === header_share) {
					header_share.classList.remove('progress-header-share--show');
				}
			});
		}
	}

	/**
	 * Secondary Menu (toggle mobile)
	 */
	var secondary_links_with_children = document.querySelectorAll('#secondary-menu .menu-item-has-children');

	secondary_links_with_children.forEach(function (link) {
		link.addEventListener('touchstart', submenuToggle, true);
	});

	function submenuToggle(event) {
		// Click on toggle
		if (event.target.parentNode.classList.contains('menu-item-has-children')) {
			event.preventDefault();
			var self = this;

			this.parentNode.children.forEach(function (link) {
				if (link !== self) {
					link.classList.remove('selected-menu');
				}
			});

			this.classList.toggle('selected-menu');
		}
	}


	/**
	 * Sliders
	 */

	var can_slide = typeof tns === 'function';

	// Home header (4 slides)
	if (!isIE && can_slide && document.querySelector('.hero-header .the-archive')) {
		tns({
			container: '.hero-header .the-archive',
			controls: false,
			loop: false,
			navPosition: 'bottom',
			slideBy: 'page',
			fixedWidth: 224,
			gutter: 8,
			responsive: {
				768: {
					fixedWidth: 300
				},
				1024: {
					disable: true
				}
			}
		});
	}

	// Home Recent Web Content (3 slides)
	if (!isIE && can_slide && document.querySelector('.section-recent-web .the-archive')) {
		tns({
			container: '.section-recent-web .the-archive',
			controls: false,
			loop: false,
			navPosition: 'bottom',
			slideBy: 'page',
			fixedWidth: 224,
			gutter: 8,
			responsive: {
				717: {
					fixedWidth: 325
				},
				1024: {
					disable: true
				}
			}
		});
	}

	// Home Next Activities (2 slides)
	if (!isIE && can_slide && document.querySelector('.section-next-activities .the-archive')) {
		tns({
			container: '.section-next-activities .the-archive',
			controls: false,
			loop: false,
			navPosition: 'bottom',
			slideBy: 'page',
			fixedWidth: 224,
			gutter: 8,
			responsive: {
				480: {
					disable: true
				}
			}
		});
	}

	// Home Recent Videos
	if (!isIE && can_slide && document.querySelector('.section-recent-videos .the-archive')) {
		tns({
			container: '.section-recent-videos .the-archive',
			controls: false,
			loop: false,
			navPosition: 'bottom',
			slideBy: 'page',
			fixedWidth: 224,
			gutter: 8,
			responsive: {
				768: {
					fixedWidth: 254
				},
				1024: {
					disable: true
				}
			}
		});
	}

	// Home Recent Specials
	if (!isIE && can_slide && document.querySelector('.section-recent-specials .the-archive--secondary')) {
		tns({
			container: '.section-recent-specials .the-archive--secondary',
			controls: false,
			loop: false,
			navPosition: 'bottom',
			slideBy: 'page',
			fixedWidth: 224,
			gutter: 8,
			responsive: {
				768: {
					fixedWidth: 254
				},
				1024: {
					disable: true
				}
			}
		});
	}

	// Post Sliders.
	var post_sliders = document.querySelectorAll('.post-slider');

	if (can_slide && post_sliders) {
		post_sliders.forEach(function (post_slider) {
			var slider = tns({
				container: post_slider.querySelector('.post-slides'),
				controls: false,
				controlsContainer: post_slider.querySelector('.slider-controls'),
				nav: true,
				navPosition: 'bottom',
				loop: false,
				rewind: true,
				slideBy: 'page',
				gutter: 8,
				items: 1,
				responsive: {
					640: {
						gutter: 20,
						items: 2
					},
					1152: {
						controls: true,
						nav: false,
						items: 3
					}
				},
				onInit: function (info) { post_slider.querySelector('.slider-controls .pages .total').textContent = info.pages; }
			});

			slider.events.on('indexChanged', function (info) {
				post_slider.querySelector('.slider-controls .pages .current').textContent = Math.round(info.index / info.items) + 1;
				post_slider.querySelector('.slider-controls .pages .total').textContent = info.pages;
			});
		});
	}

	// Categories Slide.
	var cat_slider = document.querySelector('.categories');

	if (can_slide && cat_slider) {
		tns({
			container: cat_slider.querySelector('.categories-links'),
			controlsContainer: cat_slider.querySelector('.cat-nav'),
			nav: false,
			loop: false,
			autoWidth: true,
			slideBy: 'page'
		});
	}

	// Special Annual Summaries
	var summaries_slider = document.querySelector('.annual-summaries');

	if (can_slide && summaries_slider) {
		var slider = tns({
			container: summaries_slider.querySelector('.post-slides'),
			controls: false,
			controlsContainer: summaries_slider.querySelector('.slider-controls'),
			nav: true,
			navPosition: 'bottom',
			loop: false,
			rewind: true,
			items: 1,
			slideBy: 1,
			responsive: {
				769: {
					controls: true,
					nav: false
				}
			},
			onInit: function (info) {
				summaries_slider.querySelector('.slider-controls .pages .total').textContent = info.pages;
			}
		});

		slider.events.on('indexChanged', function (info) {
			summaries_slider.querySelector('.slider-controls .pages .current').textContent = Math.round(info.index / info.items) + 1;
			summaries_slider.querySelector('.slider-controls .pages .total').textContent = info.pages;
		});
	}

	// Years Select
	var year_select = document.querySelector('.years-select');

	if (year_select) {
		year_select.querySelector('.current-year').addEventListener('click', function () {
			year_select.classList.toggle('years-select--show');
		});
	}


	/**
	 * Love me
	 */
	var love_me = document.getElementById('love_check');

	if (love_me) {
		love_me.addEventListener('change', function () {
			if (!love_me.parentNode.classList.contains('love--wait')) {
				love_me.parentNode.classList.add('love--wait');

				var request = new XMLHttpRequest();

				request.open('POST', elcano_vars.ajaxurl, true);
				request.onload = function () {
					if (this.status >= 200 && this.status < 400) {
						var resp = JSON.parse(this.response);
						document.querySelector('.love__count').textContent = resp.likes > 0 ? resp.likes : '';
						love_me.checked = resp.liked;
					} else {
						document.querySelector('.love__count').textContent = '!';
					}
					love_me.parentNode.classList.remove('love--wait');
				};
				request.onerror = function () {
					love_me.parentNode.classList.remove('love--wait');
				};
				request.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
				request.send('action=love_me&post_id=' + love_me.dataset.id + '&nonce=' + love_me.dataset.nonce);
			}
		});
	}


	/**
	 * Share/Follow links
	 */

	// Open new small centered window (secure noopener)
	function shareNewWindow(event) {
		event.preventDefault();
		var left = (screen.width - 570) / 2;
		var top = (screen.height - 570) / 2;
		var w = window.open('', '_blank', "menubar=no,toolbar=no,status=no,width=570,height=570,top=" + top + ",left=" + left);
		w.opener = null; // Secure open, remove opener
		w.location = this.href;
	}

	// Open new window (secure noopener)
	function openNewWindow(event) {
		event.preventDefault();
		window.open(this.href, '_blank', 'noopener');
	}

	// Copy Link
	function copyLink(event) {
		event.preventDefault();

		var self = this;
		var temp = document.createElement('input');
		temp.value = self.href;
		temp.style.position = 'fixed';
		temp.style.zIndex = '-100';

		document.body.appendChild(temp);
		temp.select();

		try {
			if (document.execCommand('copy')) {
				self.classList.add('social--anim');
				setTimeout(function () { self.classList.remove('social--anim'); }, 600);
			}
		} catch (err) { }

		document.body.removeChild(temp);
	}

	var share_links = document.querySelectorAll('.share-links a');

	share_links.forEach(function (share_link) {
		if (share_link.rel === 'bookmark') {
			share_link.addEventListener('click', copyLink);
		} else {
			share_link.addEventListener('click', shareNewWindow);
		}
	});

	var follow_links = document.querySelectorAll('.follow-links a');

	follow_links.forEach(function (follow_link) {
		follow_link.addEventListener('click', openNewWindow);
	});


	/**
	 * Modal Video/Images
	 */

	var can_modal = typeof BigPicture === 'function';

	var videos = document.querySelectorAll('[data-video]');

	if (can_modal && videos) {
		videos.forEach(function (video) {
			video.addEventListener('click', function (event) {
				event.preventDefault();
				var yt_id = video.dataset.video.match(/^(?:https?:)?\/\/(?:www\.)?(?:youtu\.be\/|youtube\.com(?:\/embed\/|\/v\/|\/watch\?v=|\/user\/\S+))([^\/&]{10,12})/);
				var vm_id = video.dataset.video.match(/^(?:https?:)?\/\/(?:www\.)?vimeo\.com\/(?:channels\/(?:\w+\/)?|groups\/(?:[^\/]*)\/videos\/|)(\d+)(?:|\/\?)/);

				BigPicture({
					el: video,
					vimeoSrc: vm_id && vm_id[1] || null,
					ytSrc: yt_id && yt_id[1] || null,
					ytNoCookie: true,
					dimensions: [1392, 783],
					onError: function () { alert(elcano_vars.modal_error); }
				});
			});
		});
	}

	var gallery_links = document.querySelectorAll('.blocks-gallery-grid a');

	if (can_modal && gallery_links) {
		gallery_links.forEach(function (gallery_link) {
			gallery_link.addEventListener('click', function (event) {
				event.preventDefault();
				var gallery = gallery_link.parentNode.parentNode.parentNode;

				BigPicture({
					el: gallery_link,
					gallery: gallery.querySelectorAll('a'),
					galleryAttribute: 'href',
					onError: function () { alert(elcano_vars.modal_error); }
				});
			});
		});
	}

	var image_links = document.querySelectorAll('.js-image-modal');

	if (can_modal && image_links) {
		image_links.forEach(function (image_link) {
			image_link.addEventListener('click', function (event) {
				event.preventDefault();

				BigPicture({
					el: image_link,
					imgSrc: image_link.href,
					onError: function () { alert(elcano_vars.modal_error); }
				});
			});
		});
	}


	/**
	 * Table of Contents
	 */

	if (!isIE && document.querySelector('.lwptoc')) {

		// TOC Navigation
		var toc_offset = parseInt(document.querySelector('.lwptoc').dataset.smoothScrollOffset);
		var toc_smooth = !!document.querySelector('.lwptoc').dataset.smoothScroll;
		var toc_items = document.querySelectorAll('.lwptoc .lwptoc_items a');
		var toc_disable_scroll_up;

		toc_items.forEach(function (toc_item) {
			toc_item.addEventListener('click', scrollToTarget, true);
		});

		function scrollToTarget(event) {
			event.preventDefault();
			// Disable scroll up (don't extend menu)
			capture_scroll_up = false;
			clearTimeout(toc_disable_scroll_up);
			toc_disable_scroll_up = setTimeout(function () { capture_scroll_up = true; }, 1000);

			// Scroll to node
			var el = document.getElementById(this.getAttribute('href').substr(1));
			window.scrollBy({
				top: el.getBoundingClientRect().top - toc_offset,
				behavior: toc_smooth ? 'smooth' : 'auto'
			});
		}

		// TOC Active Item
		var links = document.querySelectorAll('.lwptoc_item');
		var sections = document.querySelectorAll('span[id^="toc--"]');

		function changeTOCActive() {
			var index = sections.length;
			links.forEach(function (link) { link.classList.remove('active'); });

			while (--index && window.scrollY < sections[index].offsetTop - toc_offset) { }

			links[index].classList.add('active');
		}

		changeTOCActive();
		window.addEventListener('scroll', debounce(changeTOCActive, 25));

		// TOC Toggle (only mobile)
		document.querySelector('.lwptoc .lwptoc_items').removeAttribute('style');
		document.querySelector('.lwptoc .lwptoc_header').addEventListener('touchstart', function () {
			document.querySelector('.lwptoc .lwptoc_items').classList.toggle('lwptoc_items-visible');
		});
	}


	/**
	 * PDF Embed
	 */
	var pdfs = document.querySelectorAll('.wp-block-pdf-viewer-block-standard');

	pdfs.forEach(function (pdf) {
		var pdf_link = pdf.querySelector('.uploaded-pdf > a');
		var href = pdf_link.href || '';

		if (href != undefined && href != '') {
			var w = pdf_link.dataset.width;
			var h = pdf_link.dataset.height;
			var iframe = document.createElement('iframe');
			iframe.classList.add('pdfjs-viewer');
			iframe.style.width = isNaN(w) ? w : w + 'px';
			iframe.style.height = isNaN(h) ? h : h + 'px';
			iframe.src = elcano_vars.pdf_viewer + '?file=' + encodeURI(href);

			pdf.appendChild(iframe);
			pdf.removeChild(pdf_link.parentNode);
		}
	});


	/**
	 * Code-iframe (resize to content height)
	 */
	var code_iframe = document.querySelector('.code-iframe');

	if (code_iframe) {
		window.addEventListener('load', function () {
			code_iframe.style.height = code_iframe.contentWindow.document.documentElement.scrollHeight + 'px';
		});
	}

})();
