const express = require('express');
const router = express.Router();
const rosController = require('../controllers/rosController');

let rosC = new rosController.ROSController('ws://localhost:8080');

router
    .get('/', (req, res, next) => res.render('index', {title: 'WebApp RabbitAMR'}))
    .get('/location', (req, res, next) => rosC.getInfo(req, res, next))
    .get('/map', (req, res, next) => rosC.getMap(req, res, next))
    .get('/amcl', (req, res, next) => rosC.getAmcl(req, res, next))
    .post('/move', (req, res, next) => rosC.setCmd(req, res, next))
    .post('/landmark', (req, res, next) => rosC.setLandmark(req, res, next));

module.exports = router;
