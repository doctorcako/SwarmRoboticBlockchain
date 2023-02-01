"use strict";

/*
 GTM events
 */
window.dataLayer = window.dataLayer || [];

function dataPush(event, eventCategory, eventAction, eventLabel) {
  dataLayer.push({
    event: event,
    eventCategory: eventCategory,
    eventAction: eventAction,
    eventLabel: eventLabel
  });
}

// Clics realizados en los resultados de búsqueda
var results = document.querySelectorAll('.search-results .the-archive article .entry-title a');
if (results) results.forEach(function (result) {
  result.addEventListener('click', function (e) {
    dataPush('clicBuscador', 'Buscador', 'Clic en Resultado', this.getAttribute('href'));
  });
});

// Clics realizados en los CTAs "Anterior" y "Siguiente"
var clicSiguiente = document.getElementById('clicSiguiente');
if (clicSiguiente) clicSiguiente.addEventListener('click', function (e) {
  dataPush('clicAnteriorSiguiente', 'Lecturas', 'Siguiente', this.getAttribute('href'));
});

var clicSiguienteBottom = document.querySelector('.nav-links .nav-next > a');
if (clicSiguienteBottom) clicSiguienteBottom.addEventListener('click', function (e) {
  dataPush('clicAnteriorSiguiente', 'Lecturas', 'Siguiente', this.getAttribute('href'));
});

var clicAnterior = document.getElementById('clicAnterior');
if (clicAnterior) clicAnterior.addEventListener('click', function (e) {
  dataPush('clicAnteriorSiguiente', 'Lecturas', 'Anterior', this.getAttribute('href'));
});

var clicAnteriorBottom = document.querySelector('.nav-links .nav-previous > a');
if (clicAnteriorBottom) clicAnteriorBottom.addEventListener('click', function (e) {
  dataPush('clicAnteriorSiguiente', 'Lecturas', 'Anterior', this.getAttribute('href'));
});

// Reproducciones de podcast

// Clics en los enlaces de las tablas de contenido
var contentAnchorsPolicy = document.querySelectorAll('.policy_paper .lwptoc_itemWrap .lwptoc_item');
if (contentAnchorsPolicy) contentAnchorsPolicy.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('policyPaper', 'Policy Paper', item.textContent, '');
  });
});

var contentAnchorsWork = document.querySelectorAll('.work_document .lwptoc_itemWrap .lwptoc_item');
if (contentAnchorsWork) contentAnchorsWork.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('workDocument', 'Work Document', item.textContent, '');
  });
});

// Clics en el logo Real Instituto Elcano
document.querySelector('.site-branding .site-title').addEventListener('click', function (e) {
  dataPush('logotipo', 'Logotipo', 'Clic', 'Header');
});

document.querySelector('#footer .site-info').addEventListener('click', function (e) {
  dataPush('logotipo', 'Logotipo', 'Clic', 'Footer');
});

// Clic en categorías del supramenú
var elcanoCat = document.querySelector('.top-nav .top-tabs span');
if (elcanoCat) elcanoCat.addEventListener('click', function (e) {
  dataPush('clicSupramenu', 'Supramenú', this.textContent, '');
});

var globalCat = document.querySelector('.top-nav .top-tabs a');
if (globalCat) globalCat.addEventListener('click', function (e) {
  dataPush('clicSupramenu', 'Supramenú', this.textContent, '');
});

var languageSelectors = document.querySelectorAll('.top-languages .languages-select li');
if (languageSelectors) {
  languageSelectors.forEach(function (item) {
    item.addEventListener('click', function (e) {
      dataPush('clicSupramenu', 'Supramenú', this.textContent, '');
    });
  });
}

var subscribeSelectors = document.querySelectorAll('#top-menu .sub-menu li');
if (subscribeSelectors) {
  subscribeSelectors.forEach(function (item) {
    item.addEventListener('click', function (e) {
      dataPush('clicSupramenu', 'Supramenú', this.textContent, '');
    });
  });
}

// Clic en categorías del menú
var menuItemsFirstLevel = document.querySelectorAll('.primary-menu #primary-menu > li > a');
if (menuItemsFirstLevel) menuItemsFirstLevel.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('clicMenu', 'Menú', this.textContent, 'Nivel 1');
  });
});

var menuItemsSecondLevel = document.querySelectorAll('#primary-menu > .menu-item > .sub-menu-wrap > .sub-menu > li.menu-item > a');
if (menuItemsSecondLevel) menuItemsSecondLevel.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('clicMenu', 'Menú', this.textContent, 'Nivel 2');
  });
});

var menuItemsThirdlevel = document.querySelectorAll('#primary-menu > .menu-item > .sub-menu-wrap > .sub-menu > li.menu-item > .sub-menu-wrap .sub-menu > li > a');
if (menuItemsThirdlevel) menuItemsThirdlevel.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('clicMenu', 'Menú', this.textContent, 'Nivel 3');
  });
});

// Clic en categorías del submenú
var submenuItemsFirstLevel = document.querySelectorAll('.second-nav #secondary-menu > li.menu-item > a');
if (submenuItemsFirstLevel) submenuItemsFirstLevel.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('clicSubmenu', 'Submenú', this.textContent, 'Nivel 1');
  });
});

var submenuItemsSecondLevel = document.querySelectorAll('.second-nav #secondary-menu > li.menu-item > .sub-menu-wrap li.menu-item > a');
if (submenuItemsSecondLevel) submenuItemsSecondLevel.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('clicSubmenu', 'Submenú', this.textContent, 'Nivel 2');
  });
});

// Interacción con el footer
var footerLinks = document.querySelectorAll('.footer-links #footer-links > .menu-item > a');
if (footerLinks) footerLinks.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('clicFooter', 'Footer', this.textContent, '');
  });
});

// Like en los artículos
var love = document.getElementById('love_check');
if (love) love.addEventListener('click', function (e) {
  dataPush('likePost', 'Lecturas', 'Like', document.querySelector('h1.entry-title').textContent);
});

// Clics en los CTAs de categorías del blog
var blogCategories = document.querySelectorAll('.categories .categories-links li');
if (blogCategories) blogCategories.forEach(function (category) {
  category.addEventListener('click', function (e) {
    dataPush('blogCategory', 'Blog', category.textContent, '');
  });
});

var postCategory = document.querySelectorAll('.post-category a.category');
if (postCategory) postCategory.forEach(function (category) {
  category.addEventListener('click', function (e) {
    dataPush('blogCategory', 'Blog', category.textContent, '');
  });
});

// Clics en paginaciones
var paginationNumbers = document.querySelectorAll('.posts-pagination .page-numbers .page-numbers');
if (paginationNumbers) paginationNumbers.forEach(function (item) {
  item.addEventListener('click', function (e) {
    dataPush('clicPaginacion', 'Lecturas', 'Paginación', this.textContent);
  });
});

function getFiltersContent() {
  var filters = document.querySelectorAll('li.filter');
  var eventLabel = '';
  var typesFilter = 'Tipo de contenido: ';
  var tagFilters = ' Temas: ';
  var authorsFilters = ' Autores: ';
  var periodFilters = ' Periodo: ';

  if (filters) {
    filters.forEach(function (filter) {
      var type = filter.dataset.filter.split('__')[0];

      if (type === 'types') {
        typesFilter += filter.textContent + ' / ';
      } else if (type === 'tag' || type === 'cat') {
        tagFilters += filter.textContent + ' / ';
      } else if (type === 'authors') {
        authorsFilters += filter.textContent + ' / ';
      } else {
        periodFilters += filter.textContent + ' / ';
      }
    });
    eventLabel += typesFilter + tagFilters + authorsFilters + periodFilters;
    return eventLabel;
  } else {
    return '';
  }
}

// Clics realizados en añadir filtros
var applyFilters = document.querySelector('.selected-filters .selected-filters__apply');
if (applyFilters) applyFilters.addEventListener('click', function (e) {
  var eventLabel = getFiltersContent();
  dataPush('añadirFiltro', 'Filtro', 'Aplicar', eventLabel);
});

// Clics realizados en el CTA "limpiar todos los filtros"
var clearFilters = document.querySelector('.selected-filters .selected-filters__clear');
if (clearFilters) clearFilters.addEventListener('click', function (e) {
  var eventLabel = getFiltersContent();
  dataPush('limpiarFiltro', 'Filtro', 'Limpiar', eventLabel);
});

// Clics realizados en los CTA de eliminar alguno de los filtros aplicados
var filterss = document.querySelectorAll('li.filter');
if (filterss) filterss.forEach(function (filter) {
  filter.addEventListener('click', function (e) {
    setTimeout(function () {
      return dataPush('eliminarFiltro', 'Filtro', 'Eliminar', getFiltersContent());
    }, 1000);
  });
});
